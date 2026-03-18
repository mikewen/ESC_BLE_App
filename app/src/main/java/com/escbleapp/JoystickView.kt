package com.escbleapp

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import kotlin.math.abs

/**
 * Single-axis throttle joystick.
 *
 * ESC  mode: value 0.0 → 1.0  (bottom = stop/1000µs, top = full/2000µs)
 *            one-way only, no negative range
 * BLDC mode: value 0.0 → 1.0  (bottom = 0% duty,     top = 100% duty)
 *
 * springBack: if true, releases to 0 on finger-up (safety default)
 *             if false, holds last position (throttle-latch mode)
 */
class JoystickView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyle: Int = 0
) : View(context, attrs, defStyle) {

    // ── Public API ────────────────────────────────────────────────────────────

    var onValueChanged: ((value: Float) -> Unit)? = null

    /** 0.0 (stop) … 1.0 (full throttle) */
    var value: Float = 0f
        private set

    /** If true, finger-up springs back to 0. If false, holds position. */
    var springBack: Boolean = true

    /** Label drawn at top of joystick */
    var label: String = ""

    /** Display mode — changes label suffix and colour */
    var escMode: Boolean = true

    // ── Private state ─────────────────────────────────────────────────────────

    private var centerX = 0f
    private var centerY = 0f
    private var radius  = 0f
    private var thumbY  = 0f   // screen Y of thumb — bottom = stop, top = full
    private var activePointerId = -1

    // ── Paints ────────────────────────────────────────────────────────────────

    private val paintBase = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#1E2A3A"); style = Paint.Style.FILL
    }
    private val paintBorder = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#2E4A6E"); style = Paint.Style.STROKE; strokeWidth = 3f
    }
    private val paintTrack = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeWidth = 4f; strokeCap = Paint.Cap.ROUND
    }
    private val paintThumb = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val paintStopLine = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#66FF5252")
        style = Paint.Style.STROKE; strokeWidth = 2f
        pathEffect = DashPathEffect(floatArrayOf(8f, 6f), 0f)
    }
    private val paintLatchRing = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#AAFFB300")
        style = Paint.Style.STROKE; strokeWidth = 4f
    }
    private val paintLabel = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#88FFFFFF"); textSize = 28f; textAlign = Paint.Align.CENTER
    }
    private val paintValue = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE; textSize = 32f; textAlign = Paint.Align.CENTER
        typeface = Typeface.MONOSPACE
    }
    private val paintLockLabel = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.parseColor("#AAFFB300"); textSize = 22f; textAlign = Paint.Align.CENTER
    }

    init {
        setLayerType(LAYER_TYPE_SOFTWARE, null)
    }

    // ── Size ──────────────────────────────────────────────────────────────────

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        centerX = w / 2f
        centerY = h / 2f
        radius  = minOf(w, h) / 2f - 16f
        thumbY  = stopY()   // start at bottom = stop
    }

    /** Screen Y for value=0 (stop) — bottom of travel */
    private fun stopY() = centerY + radius
    /** Screen Y for value=1 (full) — top of travel */
    private fun fullY() = centerY - radius

    // ── Draw ──────────────────────────────────────────────────────────────────

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        // Background
        canvas.drawCircle(centerX, centerY, radius, paintBase)
        canvas.drawCircle(centerX, centerY, radius, paintBorder)

        // Stop line at bottom (value=0)
        val sy = stopY()
        canvas.drawLine(centerX - radius * 0.5f, sy, centerX + radius * 0.5f, sy, paintStopLine)

        // Active track from stop up to thumb
        val trackColor = when {
            value > 0.6f -> Color.parseColor("#FF5252")
            value > 0.3f -> Color.parseColor("#FFB300")
            value > 0.02f -> Color.parseColor("#14FFEC")
            else -> Color.parseColor("#0D7377")
        }
        paintTrack.color = trackColor
        if (value > 0.01f) {
            canvas.drawLine(centerX, sy, centerX, thumbY, paintTrack)
        }

        // Latch ring (shown when springBack=false)
        if (!springBack) {
            canvas.drawCircle(centerX, thumbY, radius * 0.32f, paintLatchRing)
        }

        // Thumb
        val thumbRadius = radius * 0.28f
        paintThumb.color = if (value > 0.02f) trackColor else Color.parseColor("#546E7A")
        val glowPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = paintThumb.color; style = Paint.Style.STROKE
            strokeWidth = thumbRadius * 0.4f; alpha = 60
            setShadowLayer(thumbRadius * 0.6f, 0f, 0f, paintThumb.color)
        }
        canvas.drawCircle(centerX, thumbY, thumbRadius * 1.2f, glowPaint)
        canvas.drawCircle(centerX, thumbY, thumbRadius, paintThumb)
        val hlPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.argb(80, 255, 255, 255); style = Paint.Style.FILL
        }
        canvas.drawCircle(centerX - thumbRadius*0.2f, thumbY - thumbRadius*0.2f, thumbRadius*0.35f, hlPaint)

        // Label (top)
        if (label.isNotEmpty()) canvas.drawText(label, centerX, 36f, paintLabel)

        // Latch indicator
        if (!springBack) canvas.drawText("LATCH", centerX, 56f, paintLockLabel)

        // Value (bottom)
        val displayStr = if (escMode) "${valueToUs(value)}µs" else "${(value * 100).toInt()}%"
        canvas.drawText(displayStr, centerX, height - 12f, paintValue)
    }

    // ── Touch ─────────────────────────────────────────────────────────────────

    @SuppressLint("ClickableViewAccessibility")
    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN, MotionEvent.ACTION_POINTER_DOWN -> {
                val idx = event.actionIndex
                activePointerId = event.getPointerId(idx)
                updateThumb(event.getY(idx))
                return true
            }
            MotionEvent.ACTION_MOVE -> {
                val idx = event.findPointerIndex(activePointerId)
                if (idx != -1) updateThumb(event.getY(idx))
                return true
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_POINTER_UP, MotionEvent.ACTION_CANCEL -> {
                if (event.getPointerId(event.actionIndex) == activePointerId) {
                    if (springBack) doSpringBack()
                    activePointerId = -1
                }
                return true
            }
        }
        return super.onTouchEvent(event)
    }

    private fun updateThumb(y: Float) {
        thumbY = y.coerceIn(fullY(), stopY())
        // Map: stopY()=0.0  fullY()=1.0
        value = ((stopY() - thumbY) / (stopY() - fullY())).coerceIn(0f, 1f)
        invalidate()
        onValueChanged?.invoke(value)
    }

    private fun doSpringBack() {
        thumbY = stopY()
        value  = 0f
        invalidate()
        onValueChanged?.invoke(0f)
    }

    // ── Public setters ────────────────────────────────────────────────────────

    /** Set value programmatically (0.0–1.0) */
    fun setValue(v: Float) {
        value  = v.coerceIn(0f, 1f)
        thumbY = stopY() - value * (stopY() - fullY())
        invalidate()
    }

    // ── Companion ─────────────────────────────────────────────────────────────

    companion object {
        /** Map 0.0–1.0 → 1000–2000 µs (ESC pulse width) */
        fun valueToUs(v: Float): Int = (1000 + v * 1000).toInt().coerceIn(1000, 2000)

        /** Map 0.0–1.0 → 0–100 duty percent (BLDC) */
        fun valueToPct(v: Float): Int = (v * 100).toInt().coerceIn(0, 100)
    }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val size = minOf(MeasureSpec.getSize(widthMeasureSpec), MeasureSpec.getSize(heightMeasureSpec))
        setMeasuredDimension(size, size)
    }
}