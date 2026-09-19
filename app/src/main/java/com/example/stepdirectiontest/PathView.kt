package com.example.stepdirectiontest

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.PointF
import android.util.AttributeSet
import android.view.View
import kotlin.math.max
import kotlin.math.min

/** 用 Canvas 顯示目前測試的相對路徑。 */
class PathView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {

    private val pathPoints = mutableListOf<PointF>()
    // 路徑較短時使用的最大單位大小；路徑較長時會自動縮小以完整顯示
    private val maxStepPixels = 40f

    private val linePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(30, 100, 220)
        style = Paint.Style.STROKE
        strokeWidth = 6f
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
    }

    private val startPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(30, 150, 80)
        style = Paint.Style.FILL
    }

    private val currentPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(220, 50, 50)
        style = Paint.Style.FILL
    }

    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.DKGRAY
        textSize = 32f
        style = Paint.Style.FILL
    }

    /** 接收 Activity 的路徑資料，並立即要求重新繪製。 */
    fun setPath(points: List<PointF>) {
        pathPoints.clear()
        points.forEach { point ->
            pathPoints.add(PointF(point.x, point.y))
        }
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        canvas.drawColor(Color.WHITE)

        if (pathPoints.isEmpty()) return

        val density = resources.displayMetrics.density
        val centerX = width / 2f
        val centerY = height / 2f
        val padding = 32f * density

        // 找出整條路徑的範圍，而不是只把最新點放在中心。
        // 這樣路徑變長時會自動縮小，起點與終點都能留在畫面中。
        val minNorth = pathPoints.minOf { it.x }
        val maxNorth = pathPoints.maxOf { it.x }
        val minEastWest = pathPoints.minOf { it.y }
        val maxEastWest = pathPoints.maxOf { it.y }
        val northRange = max(1f, (maxNorth - minNorth).toFloat())
        val eastWestRange = max(1f, (maxEastWest - minEastWest).toFloat())

        val availableWidth = max(1f, width - padding * 2f)
        val availableHeight = max(1f, height - padding * 2f)
        val fittedStepPixels = min(
            maxStepPixels * density,
            min(availableWidth / eastWestRange, availableHeight / northRange)
        )
        val stepPixels = max(1f, fittedStepPixels)
        val routeCenterNorth = (minNorth + maxNorth) / 2f
        val routeCenterEastWest = (minEastWest + maxEastWest) / 2f

        // 第二階段目前定義為：南北使用 X、東西使用 Y。
        // 繪圖時把 Y 當成左右，把 X 當成南北，並反轉畫面 Y 軸，
        // 因此「北」會往畫面上方。
        fun toCanvasPoint(point: PointF): PointF {
            val horizontal = point.y - routeCenterEastWest
            val north = point.x - routeCenterNorth
            return PointF(
                centerX + horizontal * stepPixels,
                centerY - north * stepPixels
            )
        }

        val axisPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.LTGRAY
            strokeWidth = 2f
        }

        // 如果 (0,0) 在可視範圍內，畫出簡單座標軸幫助辨認方向。
        val origin = toCanvasPoint(PointF(0f, 0f))
        if (origin.x in 0f..width.toFloat()) {
            canvas.drawLine(origin.x, 0f, origin.x, height.toFloat(), axisPaint)
        }
        if (origin.y in 0f..height.toFloat()) {
            canvas.drawLine(0f, origin.y, width.toFloat(), origin.y, axisPaint)
        }

        // 相鄰座標點用線連接，斜向座標會自然畫成斜線
        for (index in 1 until pathPoints.size) {
            val previous = toCanvasPoint(pathPoints[index - 1])
            val current = toCanvasPoint(pathPoints[index])
            canvas.drawLine(previous.x, previous.y, current.x, current.y, linePaint)
        }

        val start = toCanvasPoint(pathPoints.first())
        canvas.drawCircle(start.x, start.y, 12f, startPaint)
        canvas.drawText("S", start.x + 14f, start.y - 14f, textPaint)

        val current = toCanvasPoint(pathPoints.last())
        canvas.drawCircle(current.x, current.y, 14f, currentPaint)
    }
}
