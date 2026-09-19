package com.example.stepdirectiontest

import android.Manifest
import android.app.AlertDialog
import android.content.Context
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import java.util.Locale
import kotlin.math.abs
import kotlin.math.roundToInt

class MainActivity : AppCompatActivity(), SensorEventListener {

    // 用簡單的資料類別表示一個相對座標點
    data class Point(val x: Float, val y: Float)

    companion object {
        private const val TAG = "StepDirectionTest"
        private const val REQUEST_ACTIVITY_RECOGNITION = 1001
        // 連續五次判定相同方向後才切換，降低手機短暫晃動造成的誤判
        private const val REQUIRED_STABLE_DIRECTION_READINGS = 5
        private const val DIRECTION_HYSTERESIS_DEGREES = 8f
        private const val DIAGONAL_STEP_SIZE = 0.7f
    }

    private lateinit var sensorManager: SensorManager

    private var stepDetectorSensor: Sensor? = null
    private var stepCounterSensor: Sensor? = null
    private var rotationVectorSensor: Sensor? = null

    private lateinit var stepCountTextView: TextView
    private lateinit var directionTextView: TextView
    private lateinit var degreeTextView: TextView
    private lateinit var statusTextView: TextView
    private lateinit var currentXTextView: TextView
    private lateinit var currentYTextView: TextView
    private lateinit var pathCountTextView: TextView
    private lateinit var pathTextView: TextView
    private lateinit var movementDirectionTextView: TextView

    // 這裡用最單純的方式累加步數，方便第一階段驗證感測器是否有正常觸發
    private var currentStepCount = 0

    // 只有沒有 Step Detector 時，才使用 Step Counter 作為備援
    private var usingStepCounter = false
    private var stepCounterRegistered = false
    private var stepCounterBaseline: Int? = null
    private var lastStepCounterTotal: Int? = null

    // 第二階段：目前相對位置，起點固定為 (0, 0)
    private var currentX = 0f
    private var currentY = 0f

    // 保存目前方向，步數事件發生時使用最新方向更新座標
    private var currentDirection = "北"

    // 校正後的行走方向 = 穩定手機方向 + 八方向偏移量
    private var walkingDirectionOffsetSteps = 0
    private var isWalkingDirectionCalibrated = false
    private var movementDirection = "北"
    private var hasRealAzimuthReading = false

    private val directionNames = listOf(
        "北", "東北", "東", "東南", "南", "西南", "西", "西北"
    )

    // 方向穩定化：避免手機短暫晃動就立刻改變座標方向
    private var smoothedAzimuthDegrees: Float? = null
    private var candidateDirection = "北"
    private var candidateDirectionReadings = 0

    // 路徑會包含起點，之後每走一步就加入一個新座標
    private val pathPoints = mutableListOf<Point>()

    // 儲存目前是否正在監聽，避免重複 register
    private var isListening = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        stepCountTextView = findViewById(R.id.tvStepCount)
        directionTextView = findViewById(R.id.tvDirection)
        degreeTextView = findViewById(R.id.tvDegree)
        statusTextView = findViewById(R.id.tvStatus)
        currentXTextView = findViewById(R.id.tvCurrentX)
        currentYTextView = findViewById(R.id.tvCurrentY)
        pathCountTextView = findViewById(R.id.tvPathCount)
        pathTextView = findViewById(R.id.tvPath)
        movementDirectionTextView = findViewById(R.id.tvMovementDirection)

        findViewById<Button>(R.id.btnResetPosition).setOnClickListener {
            resetPosition()
        }

        findViewById<Button>(R.id.btnCalibrateWalkingDirection).setOnClickListener {
            showWalkingDirectionDialog()
        }

        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        stepDetectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_STEP_DETECTOR)
        stepCounterSensor = sensorManager.getDefaultSensor(Sensor.TYPE_STEP_COUNTER)
        rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

        resetPosition()
        updateInitialSupportMessage()
        updateStepText()
        updateDirectionUi(0f)
    }

    override fun onResume() {
        super.onResume()

        // 方向感測器不需要特殊權限，先處理步數權限，再統一註冊感測器
        if (needsActivityRecognitionPermission() && !hasActivityRecognitionPermission()) {
            requestActivityRecognitionPermission()
        } else {
            startSensorListening()
        }
    }

    override fun onPause() {
        super.onPause()
        stopSensorListening()
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_STEP_DETECTOR -> {
                // Step Detector 每觸發一次通常代表偵測到一步
                currentStepCount += (event.values.firstOrNull()?.roundToInt() ?: 1)
                    .coerceAtLeast(1)
                updateStepText()
                statusTextView.text = "狀態：已收到 Step Detector 步數事件"
                Log.d(TAG, "Step Detector event received")

                // 使用這一步發生當下最近一次取得的方向更新相對座標
                updatePositionByDirection()
            }

            Sensor.TYPE_STEP_COUNTER -> {
                // Step Counter 是裝置開機後的累計值，因此要先記住本次測試的起始值
                handleStepCounterEvent(event)
            }

            Sensor.TYPE_ROTATION_VECTOR -> {
                // 使用 Rotation Vector 換算成方位角，不直接使用 Gyroscope 原始數值
                val rotationMatrix = FloatArray(9)
                val remappedRotationMatrix = FloatArray(9)
                val orientationAngles = FloatArray(3)

                SensorManager.getRotationMatrixFromVector(
                    rotationMatrix,
                    event.values
                )

                // 依照目前螢幕旋轉角度重新映射座標，
                // 避免螢幕旋轉後方向偏移 90 度
                when (windowManager.defaultDisplay.rotation) {
                    android.view.Surface.ROTATION_90 -> {
                        SensorManager.remapCoordinateSystem(
                            rotationMatrix,
                            SensorManager.AXIS_Y,
                            SensorManager.AXIS_MINUS_X,
                            remappedRotationMatrix
                        )
                    }

                    android.view.Surface.ROTATION_180 -> {
                        SensorManager.remapCoordinateSystem(
                            rotationMatrix,
                            SensorManager.AXIS_MINUS_X,
                            SensorManager.AXIS_MINUS_Y,
                            remappedRotationMatrix
                        )
                    }

                    android.view.Surface.ROTATION_270 -> {
                        SensorManager.remapCoordinateSystem(
                            rotationMatrix,
                            SensorManager.AXIS_MINUS_Y,
                            SensorManager.AXIS_X,
                            remappedRotationMatrix
                        )
                    }

                    else -> {
                        rotationMatrix.copyInto(remappedRotationMatrix)
                    }
                }

                SensorManager.getOrientation(
                    remappedRotationMatrix,
                    orientationAngles
                )

                // orientationAngles[0] 是 azimuth，單位為弧度
                var azimuthDegrees = Math.toDegrees(
                    orientationAngles[0].toDouble()
                ).toFloat()

                if (azimuthDegrees < 0) {
                    azimuthDegrees += 360f
                }

                hasRealAzimuthReading = true
                updateDirectionUi(azimuthDegrees)
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // 這個簡化版本先不另外處理精度變化
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)

        if (requestCode == REQUEST_ACTIVITY_RECOGNITION) {
            if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                statusTextView.text = "狀態：已取得活動辨識權限"
                startSensorListening()
            } else {
                statusTextView.text = "狀態：未授權活動辨識，步數可能無法使用"
                startSensorListening()
            }
        }
    }

    private fun startSensorListening() {
        if (isListening) return

        val statusMessages = mutableListOf<String>()
        var registeredAnySensor = false

        if (rotationVectorSensor != null) {
            val registered = sensorManager.registerListener(
                this,
                rotationVectorSensor,
                SensorManager.SENSOR_DELAY_UI
            )
            if (registered) {
                registeredAnySensor = true
            } else {
                statusMessages.add("Rotation Vector 註冊失敗")
            }
        } else {
            statusMessages.add("此裝置不支援 Rotation Vector")
            Log.w(TAG, "Device does not support TYPE_ROTATION_VECTOR")
        }

        if (stepDetectorSensor != null) {
            if (!needsActivityRecognitionPermission() || hasActivityRecognitionPermission()) {
                usingStepCounter = false
                val registered = sensorManager.registerListener(
                    this,
                    stepDetectorSensor,
                    SensorManager.SENSOR_DELAY_UI
                )
                if (registered) {
                    registeredAnySensor = true
                    registerStepCounterBackup {
                        registeredAnySensor = true
                    }
                } else {
                    statusMessages.add("Step Detector 註冊失敗")
                    registerStepCounterFallback(statusMessages) {
                        registeredAnySensor = true
                    }
                }
            } else {
                statusMessages.add("尚未取得活動辨識權限")
            }
        } else {
            statusMessages.add("此裝置不支援 Step Detector")
            Log.w(TAG, "Device does not support TYPE_STEP_DETECTOR")
            registerStepCounterFallback(statusMessages) {
                registeredAnySensor = true
            }
        }

        if (usingStepCounter) {
            statusMessages.add("目前使用 Step Counter 備援")
        } else if (statusMessages.isEmpty()) {
            statusMessages.add("Step Detector 已註冊，請在前景連續走幾步")
        } else {
            statusMessages.add("請確認活動辨識權限，並在前景連續走幾步")
        }

        statusTextView.text = "狀態：" + statusMessages.joinToString(" / ")
        isListening = registeredAnySensor
    }

    private fun registerStepCounterFallback(
        statusMessages: MutableList<String>,
        onRegistered: () -> Unit
    ) {
        if (stepCounterSensor == null) {
            statusMessages.add("此裝置也不支援 Step Counter")
            Log.w(TAG, "Device does not support TYPE_STEP_COUNTER")
            return
        }

        val registered = sensorManager.registerListener(
            this,
            stepCounterSensor,
            SensorManager.SENSOR_DELAY_UI
        )

        if (registered) {
            usingStepCounter = true
            stepCounterRegistered = true
            stepCounterBaseline = null
            onRegistered()
        } else {
            statusMessages.add("Step Counter 註冊失敗")
        }
    }

    private fun registerStepCounterBackup(onRegistered: () -> Unit) {
        if (stepCounterSensor == null || stepCounterRegistered) return

        val registered = sensorManager.registerListener(
            this,
            stepCounterSensor,
            SensorManager.SENSOR_DELAY_UI
        )

        if (registered) {
            stepCounterRegistered = true
            stepCounterBaseline = null
            onRegistered()
            Log.d(TAG, "Step Counter backup registered")
        } else {
            Log.w(TAG, "Step Counter backup registration failed")
        }
    }

    private fun stopSensorListening() {
        if (!isListening) return

        // Activity 進入背景時解除監聽，避免在背景持續耗電
        sensorManager.unregisterListener(this)
        isListening = false
        stepCounterRegistered = false
    }

    private fun updateInitialSupportMessage() {
        val messages = mutableListOf<String>()

        if (stepDetectorSensor == null) {
            messages.add("此裝置不支援 Step Detector")
        }

        if (rotationVectorSensor == null) {
            messages.add("此裝置不支援 Rotation Vector")
        }

        statusTextView.text = if (messages.isEmpty()) {
            "狀態：等待開始監聽"
        } else {
            "狀態：" + messages.joinToString(" / ")
        }
    }

    private fun updateStepText() {
        stepCountTextView.text = "步數：$currentStepCount"
    }

    private fun handleStepCounterEvent(event: SensorEvent) {
        val totalSteps = event.values.firstOrNull()?.toInt() ?: return
        lastStepCounterTotal = totalSteps

        if (stepCounterBaseline == null) {
            stepCounterBaseline = totalSteps
            return
        }

        val stepsSinceReset = (totalSteps - stepCounterBaseline!!).coerceAtLeast(0)
        val newSteps = stepsSinceReset - currentStepCount

        if (newSteps > 0) {
            currentStepCount = stepsSinceReset

            // Step Counter 可能一次跳過多步，逐步補上每個路徑點
            repeat(newSteps) {
                updatePositionByDirection()
            }

            updateStepText()
            statusTextView.text = "狀態：已收到 Step Counter 步數事件"
            Log.d(TAG, "Step Counter event received, steps=$currentStepCount")
        }
    }

    private fun updatePositionByDirection() {
        // 每一步使用校正後的行走方向；尚未校正時會使用手機方向
        when (movementDirection) {
            "北" -> currentX += 1f
            "東北" -> {
                currentX += DIAGONAL_STEP_SIZE
                currentY += DIAGONAL_STEP_SIZE
            }
            "東" -> currentY += 1f
            "東南" -> {
                currentX -= DIAGONAL_STEP_SIZE
                currentY += DIAGONAL_STEP_SIZE
            }
            "南" -> currentX -= 1f
            "西南" -> {
                currentX -= DIAGONAL_STEP_SIZE
                currentY -= DIAGONAL_STEP_SIZE
            }
            "西" -> currentY -= 1f
            "西北" -> {
                currentX += DIAGONAL_STEP_SIZE
                currentY -= DIAGONAL_STEP_SIZE
            }
        }

        pathPoints.add(Point(currentX, currentY))
        updatePositionUi()
    }

    private fun resetPosition() {
        // 重設本次測試的步數、座標與路徑
        currentStepCount = 0
        currentX = 0f
        currentY = 0f
        stepCounterBaseline = null
        if (stepCounterRegistered) {
            stepCounterBaseline = lastStepCounterTotal
        }

        pathPoints.clear()
        pathPoints.add(Point(0f, 0f))

        updateStepText()
        updatePositionUi()
    }

    private fun updatePositionUi() {
        currentXTextView.text = "目前 X：${formatCoordinate(currentX)}"
        currentYTextView.text = "目前 Y：${formatCoordinate(currentY)}"
        pathCountTextView.text = "已記錄路徑點數：${pathPoints.size}"

        val pathText = pathPoints.joinToString(" -> ") { point ->
            "(${formatCoordinate(point.x)},${formatCoordinate(point.y)})"
        }
        pathTextView.text = "路徑：$pathText"
    }

    private fun showWalkingDirectionDialog() {
        if (!hasRealAzimuthReading || smoothedAzimuthDegrees == null) {
            statusTextView.text = "狀態：尚未取得有效手機方向，請稍候再校正"
            return
        }

        AlertDialog.Builder(this)
            .setTitle("請選擇實際行走方向")
            .setItems(directionNames.toTypedArray()) { _, selectedIndex ->
                calibrateWalkingDirection(directionNames[selectedIndex])
            }
            .setNegativeButton("取消", null)
            .show()
    }

    private fun calibrateWalkingDirection(targetDirection: String) {
        val targetIndex = directionNames.indexOf(targetDirection)
        val currentIndex = directionNames.indexOf(currentDirection)
        if (targetIndex < 0 || currentIndex < 0) return

        // 記錄實際行走方向與穩定手機方向之間相差幾個八方向
        walkingDirectionOffsetSteps =
            (targetIndex - currentIndex + directionNames.size) % directionNames.size
        isWalkingDirectionCalibrated = true
        updateMovementDirectionUi()

        statusTextView.text = "狀態：行走方向已校正為$targetDirection"
    }

    private fun updateMovementDirectionUi() {
        val phoneDirectionIndex = directionNames.indexOf(currentDirection)
        val movementDirectionIndex =
            (phoneDirectionIndex + walkingDirectionOffsetSteps) % directionNames.size
        movementDirection = directionNames[movementDirectionIndex]

        movementDirectionTextView.text = if (isWalkingDirectionCalibrated) {
            "行走方向：$movementDirection（已校正）"
        } else {
            "行走方向：$movementDirection（同手機方向）"
        }
    }

    private fun formatCoordinate(value: Float): String {
        // 座標只顯示到小數第一位，避免路徑文字太長
        return String.format(Locale.US, "%.1f", value)
    }

    private fun updateDirectionUi(azimuthDegrees: Float) {
        // 角度是圓形數值，使用最短角度差做簡單平滑，避免數字快速跳動
        val previousAzimuth = smoothedAzimuthDegrees
        val smoothed = if (previousAzimuth == null) {
            azimuthDegrees
        } else {
            val difference = shortestAngleDifference(azimuthDegrees, previousAzimuth)
            var next = previousAzimuth + difference * 0.2f
            if (next < 0f) next += 360f
            if (next >= 360f) next -= 360f
            next
        }
        smoothedAzimuthDegrees = smoothed

        val normalizedDegrees = smoothed.roundToInt().mod(360)
        val directions = listOf("北", "東北", "東", "東南", "南", "西南", "西", "西北")
        val directionIndex = ((normalizedDegrees + 22.5f) / 45f).toInt() % 8
        val detectedDirection = directions[directionIndex]

        // 方向接近邊界時，必須再多轉一小段才允許切換
        val candidateCenter = directionIndex * 45f
        val distanceFromCandidateCenter = abs(
            shortestAngleDifference(smoothed, candidateCenter)
        )
        val passedHysteresis = distanceFromCandidateCenter <=
            (22.5f - DIRECTION_HYSTERESIS_DEGREES)

        if (detectedDirection != currentDirection && !passedHysteresis) {
            // 還在容錯區內，維持上一個方向
            candidateDirectionReadings = 0
        } else if (detectedDirection == candidateDirection) {
            candidateDirectionReadings++
        } else {
            candidateDirection = detectedDirection
            candidateDirectionReadings = 1
        }

        // 新方向連續穩定出現後才切換
        if (candidateDirectionReadings >= REQUIRED_STABLE_DIRECTION_READINGS) {
            currentDirection = candidateDirection
        }

        directionTextView.text = "目前方向：$currentDirection"
        degreeTextView.text = "目前角度：${normalizedDegrees}°"
        updateMovementDirectionUi()
    }

    private fun shortestAngleDifference(from: Float, to: Float): Float {
        var difference = from - to
        if (difference > 180f) difference -= 360f
        if (difference < -180f) difference += 360f
        return difference
    }

    private fun needsActivityRecognitionPermission(): Boolean {
        return Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q
    }

    private fun hasActivityRecognitionPermission(): Boolean {
        return ContextCompat.checkSelfPermission(
            this,
            Manifest.permission.ACTIVITY_RECOGNITION
        ) == PackageManager.PERMISSION_GRANTED
    }

    private fun requestActivityRecognitionPermission() {
        ActivityCompat.requestPermissions(
            this,
            arrayOf(Manifest.permission.ACTIVITY_RECOGNITION),
            REQUEST_ACTIVITY_RECOGNITION
        )
    }
}
