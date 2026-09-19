package com.example.stepdirectiontest

import android.Manifest
import android.app.AlertDialog
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.PointF
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
import kotlin.math.abs
import kotlin.math.roundToInt
import kotlin.math.sqrt

class MainActivity : AppCompatActivity(), SensorEventListener {

    // 用簡單的資料類別表示一個相對座標點
    data class Point(val x: Int, val y: Int)

    companion object {
        private const val TAG = "StepDirectionTest"
        private const val REQUEST_ACTIVITY_RECOGNITION = 1001
        // 新方向要連續出現幾次，才允許 lockedDirection 切換
        // 2 次可以縮短轉彎延遲，同時仍可避免單次感測雜訊立即切換
        private const val DIRECTION_CONFIRM_COUNT = 2
        // 四方位每個區間是 90 度；越過 45 度邊界約 15 度後才切換方向
        private const val DIRECTION_HYSTERESIS_DEGREES = 15f
        // 數值越大，角度平滑反應越快；太大可能增加晃動造成的方向切換
        private const val DIRECTION_SMOOTHING_FACTOR = 0.35f
        // 加速度補充偵測的門檻，單位約為 m/s^2；之後可依手機靈敏度調整
        // 提高門檻，避免手持手機的小幅震動被當成一步
        private const val ACCELERATION_THRESHOLD = 1.6f
        // 放寬波峰下降判斷，避免正常走路的訊號一直沒有回到很低的值
        private const val ACCELERATION_RELEASE_THRESHOLD = 0.9f
        // 正常走路時，兩個步伐不應太靠近，避免一次晃動被算成多步
        private const val MIN_ACCELERATION_STEP_INTERVAL_MS = 450L
        // 等待短時間讓 Step Detector 與加速度候選可以合併去重
        private const val STEP_FUSION_WINDOW_MS = 220L
        // 手機正在快速旋轉時，不接受加速度補充候選
        // 只排除較快速的原地轉手機，避免一般走路擺動被誤判成旋轉
        private const val ROTATION_IGNORE_THRESHOLD = 1.2f
        private const val ROTATION_IGNORE_WINDOW_MS = 150L
        // 旋轉結束後短暫忽略殘留上下晃動，避免轉身被多算一步
        private const val ROTATION_COOLDOWN_MS = 350L
        private const val NANOS_PER_MILLISECOND = 1_000_000L
    }

    private lateinit var sensorManager: SensorManager

    private var stepDetectorSensor: Sensor? = null
    private var stepCounterSensor: Sensor? = null
    private var rotationVectorSensor: Sensor? = null
    private var accelerometerSensor: Sensor? = null
    private var gyroscopeSensor: Sensor? = null

    private lateinit var stepCountTextView: TextView
    private lateinit var directionTextView: TextView
    private lateinit var degreeTextView: TextView
    private lateinit var statusTextView: TextView
    private lateinit var currentXTextView: TextView
    private lateinit var currentYTextView: TextView
    private lateinit var pathCountTextView: TextView
    private lateinit var pathTextView: TextView
    private lateinit var lastStepDirectionTextView: TextView
    private lateinit var movementDirectionTextView: TextView
    private lateinit var recordingStatusTextView: TextView
    private lateinit var pathView: PathView

    // 這裡用最單純的方式累加步數，方便第一階段驗證感測器是否有正常觸發
    private var currentStepCount = 0

    // 只有沒有 Step Detector 時，才使用 Step Counter 作為備援
    private var usingStepCounter = false
    private var stepCounterRegistered = false
    private var stepCounterBaseline: Int? = null
    private var lastStepCounterTotal: Int? = null

    // 混合步數偵測：加速度事件先放入候選清單，稍後和系統步數事件合併
    private val pendingStepCandidates = mutableListOf<Long>()
    private var lastAcceptedStepTimestampNs = Long.MIN_VALUE
    private var lastAccelerationPeakTimestampNs = Long.MIN_VALUE
    private var accelerationWasAboveThreshold = false
    private var accelerationPeakTimestampNs = Long.MIN_VALUE
    private var accelerationPeakValue = 0f
    private val gravity = FloatArray(3)
    private var hasGravityEstimate = false
    private var latestAngularSpeed = 0f
    private var latestGyroscopeTimestampNs = Long.MIN_VALUE
    private var lastRotationDetectedTimestampNs = Long.MIN_VALUE

    // 第二階段：目前相對位置，起點固定為 (0, 0)
    private var currentX = 0
    private var currentY = 0

    // 手機方向：用來顯示目前手機朝向，也作為校正的基準
    private var currentDirection = "北"

    // 鎖定方向：只有穩定確認後才更新，步數與路徑只使用這個方向
    private var lockedDirection = "北"
    private var lockedDirectionCandidate = "北"
    private var lockedDirectionCandidateReadings = 0

    // 校正後的行走方向 = 穩定手機方向 + 四方向偏移量
    private var walkingDirectionOffsetSteps = 0
    private var isWalkingDirectionCalibrated = false
    private var hasRealAzimuthReading = false
    private var isRecording = false

    private val directionNames = listOf("北", "東", "南", "西")

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
        lastStepDirectionTextView = findViewById(R.id.tvLastStepDirection)
        movementDirectionTextView = findViewById(R.id.tvMovementDirection)
        recordingStatusTextView = findViewById(R.id.tvRecordingStatus)
        pathView = findViewById(R.id.pathView)

        findViewById<Button>(R.id.btnResetPosition).setOnClickListener {
            resetPosition()
        }

        findViewById<Button>(R.id.btnCalibrateWalkingDirection).setOnClickListener {
            showWalkingDirectionDialog()
        }

        findViewById<Button>(R.id.btnStartRecording).setOnClickListener {
            startRecording()
        }

        findViewById<Button>(R.id.btnStopRecording).setOnClickListener {
            stopRecording()
        }

        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        stepDetectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_STEP_DETECTOR)
        stepCounterSensor = sensorManager.getDefaultSensor(Sensor.TYPE_STEP_COUNTER)
        rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
        accelerometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscopeSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

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
                if (isPhoneRotating(event.timestamp)) {
                    Log.d(TAG, "Ignore Step Detector event while phone is rotating")
                    return
                }
                statusTextView.text = "狀態：已收到 Step Detector 步數事件"
                Log.d(TAG, "Step Detector event received")
                enqueueStepCandidate(event.timestamp)
            }

            Sensor.TYPE_STEP_COUNTER -> {
                // Step Counter 是裝置開機後的累計值，因此要先記住本次測試的起始值
                handleStepCounterEvent(event)
            }

            Sensor.TYPE_ACCELEROMETER -> {
                // Step Detector 沒抓到時，使用加速度波峰產生補充候選
                handleAccelerometerEvent(event)
            }

            Sensor.TYPE_GYROSCOPE -> {
                handleGyroscopeEvent(event)
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
                SensorManager.SENSOR_DELAY_GAME
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
                    SensorManager.SENSOR_DELAY_GAME
                )
                if (registered) {
                    registeredAnySensor = true
                    // Step Detector 已經註冊成功時，不再同時註冊 Step Counter。
                    // 避免兩個來源交錯更新同一個步數，造成重設後步數被忽略。
                    Log.d(TAG, "Using Step Detector as the only step sensor")

                    // 加速度計只作為 Step Detector 漏步時的補充來源。
                    if (accelerometerSensor != null) {
                        val accelerationRegistered = sensorManager.registerListener(
                            this,
                            accelerometerSensor,
                            SensorManager.SENSOR_DELAY_GAME
                        )
                        if (!accelerationRegistered) {
                            statusMessages.add("加速度計註冊失敗")
                        } else {
                            Log.d(TAG, "Using accelerometer as a supplemental step source")
                        }
                    } else {
                        statusMessages.add("此裝置不支援加速度計")
                    }

                    // 陀螺儀只用來排除轉手機造成的加速度假步數，不拿來當方向值。
                    if (gyroscopeSensor != null) {
                        sensorManager.registerListener(
                            this,
                            gyroscopeSensor,
                            SensorManager.SENSOR_DELAY_GAME
                        )
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
            statusMessages.add("Step Detector + 加速度補充已註冊，請在前景連續走幾步")
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
            SensorManager.SENSOR_DELAY_GAME
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

    /**
     * 用簡單的重力濾除與波峰判斷找出加速度步伐候選。
     * 這裡不直接增加步數，必須先經過下面的候選合併與去重。
     */
    private fun handleAccelerometerEvent(event: SensorEvent) {
        processPendingStepCandidates(event.timestamp)

        if (!hasGravityEstimate) {
            event.values.copyInto(gravity, endIndex = 3)
            hasGravityEstimate = true
            return
        }

        // 用低通方式估計重力，再從原始加速度扣除重力。
        val gravityFactor = 0.8f
        val movementFactor = 1f - gravityFactor
        for (index in 0..2) {
            gravity[index] = gravityFactor * gravity[index] +
                movementFactor * event.values[index]
        }

        val linearX = event.values[0] - gravity[0]
        val linearY = event.values[1] - gravity[1]
        val linearZ = event.values[2] - gravity[2]
        val movementAcceleration = sqrt(
            linearX * linearX + linearY * linearY + linearZ * linearZ
        )

        // 手機正在旋轉時，清除這一波候選，避免原地轉向被算成一步。
        if (isPhoneRotating(event.timestamp)) {
            accelerationWasAboveThreshold = false
            accelerationPeakTimestampNs = Long.MIN_VALUE
            accelerationPeakValue = 0f
            return
        }

        // 必須完整經過「上升到波峰，再下降」才建立候選，
        // 不再於剛超過門檻的瞬間直接計算步數。
        if (!accelerationWasAboveThreshold &&
            movementAcceleration >= ACCELERATION_THRESHOLD
        ) {
            accelerationWasAboveThreshold = true
            accelerationPeakTimestampNs = event.timestamp
            accelerationPeakValue = movementAcceleration
        } else if (accelerationWasAboveThreshold) {
            if (movementAcceleration > accelerationPeakValue) {
                accelerationPeakValue = movementAcceleration
                accelerationPeakTimestampNs = event.timestamp
            }

            if (movementAcceleration <= ACCELERATION_RELEASE_THRESHOLD) {
                accelerationWasAboveThreshold = false
                val minimumIntervalNs =
                    MIN_ACCELERATION_STEP_INTERVAL_MS * NANOS_PER_MILLISECOND
                val hasEnoughTimeSinceLastPeak =
                    lastAccelerationPeakTimestampNs == Long.MIN_VALUE ||
                        accelerationPeakTimestampNs - lastAccelerationPeakTimestampNs >=
                        minimumIntervalNs

                if (accelerationPeakTimestampNs != Long.MIN_VALUE &&
                    hasEnoughTimeSinceLastPeak
                ) {
                    lastAccelerationPeakTimestampNs = accelerationPeakTimestampNs
                    enqueueStepCandidate(accelerationPeakTimestampNs)
                }

                accelerationPeakTimestampNs = Long.MIN_VALUE
                accelerationPeakValue = 0f
            }
        }
    }

    /** 記錄目前旋轉速度；只作為加速度補充的排除條件。 */
    private fun handleGyroscopeEvent(event: SensorEvent) {
        latestAngularSpeed = sqrt(
            event.values[0] * event.values[0] +
                event.values[1] * event.values[1] +
                event.values[2] * event.values[2]
        )
        latestGyroscopeTimestampNs = event.timestamp
        if (latestAngularSpeed >= ROTATION_IGNORE_THRESHOLD) {
            lastRotationDetectedTimestampNs = event.timestamp
        }
    }

    private fun isPhoneRotating(timestampNs: Long): Boolean {
        if (latestGyroscopeTimestampNs == Long.MIN_VALUE) return false
        val timeSinceGyroscopeNs = abs(timestampNs - latestGyroscopeTimestampNs)
        val ignoreWindowNs = ROTATION_IGNORE_WINDOW_MS * NANOS_PER_MILLISECOND
        val cooldownNs = ROTATION_COOLDOWN_MS * NANOS_PER_MILLISECOND
        val isRecentRotationSample = timeSinceGyroscopeNs <= ignoreWindowNs &&
            latestAngularSpeed >= ROTATION_IGNORE_THRESHOLD
        val isInRotationCooldown = lastRotationDetectedTimestampNs != Long.MIN_VALUE &&
            timestampNs >= lastRotationDetectedTimestampNs &&
            timestampNs - lastRotationDetectedTimestampNs <= cooldownNs
        return isRecentRotationSample || isInRotationCooldown
    }

    /** 把系統步數與加速度候選放在同一個時間窗內去重。 */
    private fun enqueueStepCandidate(timestampNs: Long) {
        processPendingStepCandidates(timestampNs)

        val fusionWindowNs = STEP_FUSION_WINDOW_MS * NANOS_PER_MILLISECOND
        if (lastAcceptedStepTimestampNs != Long.MIN_VALUE &&
            abs(timestampNs - lastAcceptedStepTimestampNs) <= fusionWindowNs
        ) {
            return
        }

        if (pendingStepCandidates.any {
                abs(timestampNs - it) <= fusionWindowNs
            }
        ) {
            return
        }

        pendingStepCandidates.add(timestampNs)
        pendingStepCandidates.sort()
    }

    /** 候選事件經過短暫等待後，才正式算成一步。 */
    private fun processPendingStepCandidates(nowTimestampNs: Long) {
        val fusionWindowNs = STEP_FUSION_WINDOW_MS * NANOS_PER_MILLISECOND
        val minimumStepIntervalNs =
            MIN_ACCELERATION_STEP_INTERVAL_MS * NANOS_PER_MILLISECOND

        while (pendingStepCandidates.isNotEmpty()) {
            val candidateTimestampNs = pendingStepCandidates.first()
            if (nowTimestampNs - candidateTimestampNs < fusionWindowNs) {
                break
            }

            pendingStepCandidates.removeAll {
                abs(it - candidateTimestampNs) <= fusionWindowNs
            }

            if (lastAcceptedStepTimestampNs == Long.MIN_VALUE ||
                candidateTimestampNs - lastAcceptedStepTimestampNs >=
                minimumStepIntervalNs
            ) {
                acceptStep(candidateTimestampNs)
            }
        }
    }

    /** 正式增加步數，並在記錄模式下同步加入座標與 Canvas 路徑。 */
    private fun acceptStep(timestampNs: Long) {
        lastAcceptedStepTimestampNs = timestampNs
        currentStepCount++
        updateStepText()
        statusTextView.text = "狀態：已確認一步（系統或加速度感測器）"
        Log.d(TAG, "Step accepted, steps=$currentStepCount")

        if (isRecording) {
            updatePositionByDirection()
        }
    }

    private fun updateStepText() {
        stepCountTextView.text = "步數：$currentStepCount"
    }

    private fun handleStepCounterEvent(event: SensorEvent) {
        // 只有真的使用 Step Counter 備援時，才處理這類事件。
        if (!usingStepCounter) return

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

            if (isRecording) {
                // Step Counter 可能一次跳過多步，逐步補上每個路徑點
                repeat(newSteps) {
                    updatePositionByDirection()
                }
            }

            updateStepText()
            statusTextView.text = "狀態：已收到 Step Counter 步數事件"
            Log.d(TAG, "Step Counter event received, steps=$currentStepCount")
        }
    }

    private fun updatePositionByDirection() {
        // 每一步只使用已穩定鎖定的方向，避免瞬間角度晃動改變路徑
        when (lockedDirection) {
            "北" -> currentX += 1
            "東" -> currentY += 1
            "南" -> currentX -= 1
            "西" -> currentY -= 1
        }

        // 顯示這一步真正採用的方向，方便確認轉彎是否已被鎖定
        lastStepDirectionTextView.text = "最後一步方向：$lockedDirection"
        Log.d(TAG, "Path step direction=$lockedDirection, position=($currentX,$currentY)")

        pathPoints.add(Point(currentX, currentY))
        updatePositionUi()
        updatePathView()
    }

    private fun resetPosition() {
        // 重設本次測試的步數、座標與路徑
        currentStepCount = 0
        resetStepFusionState()
        currentX = 0
        currentY = 0
        stepCounterBaseline = null
        if (stepCounterRegistered) {
            stepCounterBaseline = lastStepCounterTotal
        }

        pathPoints.clear()
        pathPoints.add(Point(0, 0))

        // 新測試不要沿用上一段路徑最後的方向，改用目前校正後的方向開始。
        val detectedMovementDirection = detectCurrentMovementDirection()
        if (detectedMovementDirection != null) {
            lockedDirection = detectedMovementDirection
            lockedDirectionCandidate = detectedMovementDirection
            lockedDirectionCandidateReadings = 0
        }

        updateStepText()
        updatePositionUi()
        lastStepDirectionTextView.text = "最後一步方向：尚未記錄"
        updateMovementDirectionUi()
        updatePathView()
    }

    /** 重設混合步數偵測的暫存狀態，不影響方向校正。 */
    private fun resetStepFusionState() {
        pendingStepCandidates.clear()
        lastAcceptedStepTimestampNs = Long.MIN_VALUE
        lastAccelerationPeakTimestampNs = Long.MIN_VALUE
        accelerationWasAboveThreshold = false
        accelerationPeakTimestampNs = Long.MIN_VALUE
        accelerationPeakValue = 0f
        hasGravityEstimate = false
        latestAngularSpeed = 0f
        latestGyroscopeTimestampNs = Long.MIN_VALUE
        lastRotationDetectedTimestampNs = Long.MIN_VALUE
    }

    private fun updatePositionUi() {
        currentXTextView.text = "目前 X：$currentX"
        currentYTextView.text = "目前 Y：$currentY"
        pathCountTextView.text = "已記錄路徑點數：${pathPoints.size}"

        val pathText = pathPoints.joinToString(" -> ") { point ->
            "(${point.x},${point.y})"
        }
        pathTextView.text = "路徑：$pathText"
    }

    private fun updatePathView() {
        val canvasPoints = pathPoints.map { point ->
            PointF(point.x.toFloat(), point.y.toFloat())
        }
        pathView.setPath(canvasPoints)
    }

    private fun startRecording() {
        // 開始新的路徑測試，起點重新設為 (0, 0)
        resetPosition()
        isRecording = true
        recordingStatusTextView.text = "記錄狀態：記錄中"
        statusTextView.text = "狀態：開始記錄，請開始走路"
    }

    private fun stopRecording() {
        // 停止修改座標，但 sensor 仍然繼續顯示步數與方向
        isRecording = false
        recordingStatusTextView.text = "記錄狀態：已停止"
        statusTextView.text = "狀態：已停止路徑記錄，感測器仍在監聽"
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

        // 記錄實際行走方向與穩定手機方向之間相差幾個四方向
        walkingDirectionOffsetSteps =
            (targetIndex - currentIndex + directionNames.size) % directionNames.size
        isWalkingDirectionCalibrated = true
        lockedDirection = targetDirection
        lockedDirectionCandidate = targetDirection
        lockedDirectionCandidateReadings = 0
        updateMovementDirectionUi()

        statusTextView.text = "狀態：行走方向已校正為$targetDirection"
    }

    private fun updateMovementDirectionUi() {
        val phoneAzimuth = smoothedAzimuthDegrees ?: 0f
        val movementAzimuth = normalizeAngle(
            phoneAzimuth + walkingDirectionOffsetSteps * 90f
        )
        val movementDirectionIndex = directionIndexForPath(movementAzimuth)
        val detectedMovementDirection = directionNames[movementDirectionIndex]

        updateLockedDirection(detectedMovementDirection, movementAzimuth)

        movementDirectionTextView.text = if (isWalkingDirectionCalibrated) {
            "行走方向（鎖定）：$lockedDirection（已校正）"
        } else {
            "行走方向（鎖定）：$lockedDirection（同手機方向）"
        }
    }

    // 取得目前「校正後」的行走方向，供重設測試時初始化 lockedDirection。
    private fun detectCurrentMovementDirection(): String? {
        val phoneAzimuth = smoothedAzimuthDegrees ?: return null
        val movementAzimuth = normalizeAngle(
            phoneAzimuth + walkingDirectionOffsetSteps * 90f
        )
        return directionNames[directionIndexForPath(movementAzimuth)]
    }

    private fun updateLockedDirection(
        detectedDirection: String,
        movementAzimuth: Float
    ) {
        if (detectedDirection == lockedDirection) {
            lockedDirectionCandidateReadings = 0
            lockedDirectionCandidate = lockedDirection
            return
        }

        val candidateIndex = directionNames.indexOf(detectedDirection)
        val candidateCenter = candidateIndex * 90f
        val distanceFromCandidateCenter = abs(
            shortestAngleDifference(movementAzimuth, candidateCenter)
        )
        val passedHysteresis = distanceFromCandidateCenter <=
            (45f - DIRECTION_HYSTERESIS_DEGREES)

        if (!passedHysteresis) {
            // 還在邊界容錯區，維持原本 lockedDirection
            lockedDirectionCandidateReadings = 0
            return
        }

        if (detectedDirection == lockedDirectionCandidate) {
            lockedDirectionCandidateReadings++
        } else {
            lockedDirectionCandidate = detectedDirection
            lockedDirectionCandidateReadings = 1
        }

        // 新方向連續穩定出現後，才真正更新 lockedDirection
        if (lockedDirectionCandidateReadings >= DIRECTION_CONFIRM_COUNT) {
            lockedDirection = lockedDirectionCandidate
            lockedDirectionCandidateReadings = 0
        }
    }

    private fun updateDirectionUi(azimuthDegrees: Float) {
        // 角度是圓形數值，使用最短角度差做簡單平滑，避免數字快速跳動
        val previousAzimuth = smoothedAzimuthDegrees
        val smoothed = if (previousAzimuth == null) {
            azimuthDegrees
        } else {
            val difference = shortestAngleDifference(azimuthDegrees, previousAzimuth)
            var next = previousAzimuth + difference * DIRECTION_SMOOTHING_FACTOR
            if (next < 0f) next += 360f
            if (next >= 360f) next -= 360f
            next
        }
        smoothedAzimuthDegrees = smoothed

        val directions = directionNames
        val directionIndex = directionIndexForPath(smoothed)
        val detectedDirection = directions[directionIndex]

        // 方向接近邊界時，必須再多轉一小段才允許切換
        val candidateCenter = directionIndex * 90f
        val distanceFromCandidateCenter = abs(
            shortestAngleDifference(smoothed, candidateCenter)
        )
        val passedHysteresis = distanceFromCandidateCenter <=
            (45f - DIRECTION_HYSTERESIS_DEGREES)

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
        if (candidateDirectionReadings >= DIRECTION_CONFIRM_COUNT) {
            currentDirection = candidateDirection
        }

        directionTextView.text = "目前方向：$currentDirection"
        // 角度欄位保留原始感測角度，路徑則不直接使用這個瞬間值
        val rawDegrees = normalizeAngle(azimuthDegrees).roundToInt().mod(360)
        degreeTextView.text = "目前角度：${rawDegrees}°"
        updateMovementDirectionUi()
    }

    private fun normalizeAngle(angle: Float): Float {
        return (angle % 360f + 360f) % 360f
    }

    private fun directionIndexForPath(azimuthDegrees: Float): Int {
        // 四方向各約 90 度，搭配 hysteresis 降低邊界來回切換。
        val degrees = (azimuthDegrees % 360f + 360f) % 360f
        return when {
            degrees >= 315f || degrees < 45f -> 0 // 北
            degrees < 135f -> 1                    // 東
            degrees < 225f -> 2                    // 南
            else -> 3                              // 西
        }
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
