package com.escbleapp

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.View
import kotlin.math.*

/**
 * Minimal compass rose — draws a filled needle pointing to [headingDeg].
 * Dark background, teal needle, cardinal labels.
 */
class CompassView @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null, defStyle: Int = 0
) : View(context, attrs, defStyle) {

    var headingDeg: Float = 0f
        set(v) { field = v; invalidate() }

    var hasFix: Boolean = false
        set(v) { field = v; invalidate() }

    private val paintBg = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#111E35"); style = Paint.Style.FILL
    }
    private val paintRing = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#1A2E4A"); style = Paint.Style.STROKE; strokeWidth = 3f
    }
    private val paintNeedle = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }
    private val paintLabel = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#88AABBCC"); textAlign = Paint.Align.CENTER
        typeface = Typeface.MONOSPACE
    }
    private val paintN = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textAlign = Paint.Align.CENTER; typeface = Typeface.create(Typeface.MONOSPACE, Typeface.BOLD)
    }
    private val paintTick = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#334455"); style = Paint.Style.STROKE; strokeWidth = 1.5f
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val cx = width / 2f
        val cy = height / 2f
        val r  = minOf(cx, cy) - 4f

        // Background
        canvas.drawCircle(cx, cy, r, paintBg)
        canvas.drawCircle(cx, cy, r, paintRing)

        // Tick marks every 30°
        for (i in 0 until 12) {
            val angle = Math.toRadians(i * 30.0)
            val inner = r * 0.82f
            val outer = r * 0.95f
            canvas.drawLine(
                cx + (inner * sin(angle)).toFloat(), cy - (inner * cos(angle)).toFloat(),
                cx + (outer * sin(angle)).toFloat(), cy - (outer * cos(angle)).toFloat(),
                paintTick
            )
        }

        // Cardinal labels
        paintLabel.textSize = r * 0.22f
        paintN.textSize     = r * 0.24f
        val cardinals = listOf("N" to 0f, "E" to 90f, "S" to 180f, "W" to 270f)
        for ((label, deg) in cardinals) {
            val angle = Math.toRadians(deg.toDouble())
            val lx = cx + (r * 0.62f * sin(angle)).toFloat()
            val ly = cy - (r * 0.62f * cos(angle)).toFloat() + paintLabel.textSize * 0.35f
            if (label == "N") {
                paintN.color = if (hasFix) Color.parseColor("#FF5252") else Color.parseColor("#663333")
                canvas.drawText(label, lx, ly, paintN)
            } else {
                paintLabel.color = if (hasFix) Color.parseColor("#88AABBCC") else Color.parseColor("#334455")
                canvas.drawText(label, lx, ly, paintLabel)
            }
        }

        // Needle
        if (hasFix) {
            canvas.save()
            canvas.rotate(headingDeg, cx, cy)

            // Forward tip (teal)
            paintNeedle.color = Color.parseColor("#14FFEC")
            val needlePath = Path().apply {
                moveTo(cx, cy - r * 0.70f)         // tip
                lineTo(cx - r * 0.10f, cy + r * 0.15f)
                lineTo(cx, cy + r * 0.08f)
                lineTo(cx + r * 0.10f, cy + r * 0.15f)
                close()
            }
            canvas.drawPath(needlePath, paintNeedle)

            // Rear half (dimmed)
            paintNeedle.color = Color.parseColor("#334455")
            val rearPath = Path().apply {
                moveTo(cx, cy + r * 0.08f)
                lineTo(cx - r * 0.10f, cy + r * 0.15f)
                lineTo(cx, cy + r * 0.55f)
                lineTo(cx + r * 0.10f, cy + r * 0.15f)
                close()
            }
            canvas.drawPath(rearPath, paintNeedle)
            canvas.restore()
        } else {
            // No fix — draw a dim X
            paintTick.strokeWidth = 2.5f
            val d = r * 0.3f
            canvas.drawLine(cx - d, cy - d, cx + d, cy + d, paintTick)
            canvas.drawLine(cx + d, cy - d, cx - d, cy + d, paintTick)
        }

        // Centre dot
        paintNeedle.color = Color.WHITE
        canvas.drawCircle(cx, cy, r * 0.07f, paintNeedle)
    }

    override fun onMeasure(w: Int, h: Int) {
        val size = minOf(MeasureSpec.getSize(w), MeasureSpec.getSize(h))
        setMeasuredDimension(size, size)
    }
}
