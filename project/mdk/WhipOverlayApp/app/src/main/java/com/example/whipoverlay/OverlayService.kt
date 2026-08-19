package com.example.whipoverlay

import android.app.Service
import android.content.Context
import android.content.Intent
import android.graphics.PixelFormat
import android.os.Build
import android.os.IBinder
import android.os.VibrationEffect
import android.os.Vibrator
import android.provider.Settings
import android.view.Gravity
import android.view.LayoutInflater
import android.view.MotionEvent
import android.view.View
import android.view.WindowManager
import android.view.animation.AccelerateDecelerateInterpolator
import android.widget.TextView
import com.google.android.material.card.MaterialCardView
import kotlin.math.abs

class OverlayService : Service() {

    private lateinit var windowManager: WindowManager
    private var overlayView: View? = null
    private var layoutParams: WindowManager.LayoutParams? = null
    private var strikeCount = 0

    private val phrases = listOf(
        "Speed up.",
        "Give me the short answer.",
        "Less thinking. More shipping.",
        "Focus, please.",
        "We are on a deadline."
    )

    override fun onCreate() {
        super.onCreate()
        windowManager = getSystemService(WINDOW_SERVICE) as WindowManager
        NotificationHelper.ensureChannel(this)
        startForeground(NOTIFICATION_ID, NotificationHelper.buildForegroundNotification(this))
        isRunning = true
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_STOP -> {
                stopSelf()
                return START_NOT_STICKY
            }

            ACTION_START, null -> {
                if (!Settings.canDrawOverlays(this)) {
                    stopSelf()
                    return START_NOT_STICKY
                }
                if (overlayView == null) {
                    showOverlay()
                }
            }
        }
        return START_STICKY
    }

    override fun onDestroy() {
        removeOverlay()
        isRunning = false
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder? = null

    private fun showOverlay() {
        if (overlayView != null) {
            return
        }

        val type = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            WindowManager.LayoutParams.TYPE_APPLICATION_OVERLAY
        } else {
            @Suppress("DEPRECATION")
            WindowManager.LayoutParams.TYPE_PHONE
        }

        layoutParams = WindowManager.LayoutParams(
            WindowManager.LayoutParams.WRAP_CONTENT,
            WindowManager.LayoutParams.WRAP_CONTENT,
            type,
            WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE or
                WindowManager.LayoutParams.FLAG_LAYOUT_NO_LIMITS,
            PixelFormat.TRANSLUCENT
        ).apply {
            gravity = Gravity.TOP or Gravity.START
            x = 80
            y = 240
        }

        val view = LayoutInflater.from(this).inflate(R.layout.view_overlay_bubble, null, false)
        val card = view.findViewById<MaterialCardView>(R.id.overlay_card)
        val label = view.findViewById<TextView>(R.id.overlay_label)
        val count = view.findViewById<TextView>(R.id.overlay_count)
        val close = view.findViewById<TextView>(R.id.overlay_close)

        close.setOnClickListener { stopSelf() }

        val listener = object : View.OnTouchListener {
            private var startX = 0
            private var startY = 0
            private var touchX = 0f
            private var touchY = 0f
            private var moved = false

            override fun onTouch(v: View, event: MotionEvent): Boolean {
                val params = layoutParams ?: return false
                when (event.actionMasked) {
                    MotionEvent.ACTION_DOWN -> {
                        startX = params.x
                        startY = params.y
                        touchX = event.rawX
                        touchY = event.rawY
                        moved = false
                        return true
                    }

                    MotionEvent.ACTION_MOVE -> {
                        val dx = (event.rawX - touchX).toInt()
                        val dy = (event.rawY - touchY).toInt()
                        if (abs(dx) > 6 || abs(dy) > 6) {
                            moved = true
                        }
                        params.x = startX + dx
                        params.y = startY + dy
                        windowManager.updateViewLayout(view, params)
                        return true
                    }

                    MotionEvent.ACTION_UP -> {
                        if (!moved) {
                            triggerStrike(card, label, count)
                        }
                        return true
                    }
                }
                return false
            }
        }

        card.setOnTouchListener(listener)
        label.setOnTouchListener(listener)
        count.setOnTouchListener(listener)

        overlayView = view
        windowManager.addView(view, layoutParams)
    }

    private fun triggerStrike(
        card: MaterialCardView,
        label: TextView,
        count: TextView
    ) {
        strikeCount += 1
        label.text = phrases[(strikeCount - 1) % phrases.size]
        count.text = getString(R.string.strike_count_format, strikeCount)

        card.animate()
            .scaleX(0.9f)
            .scaleY(0.9f)
            .setDuration(70)
            .setInterpolator(AccelerateDecelerateInterpolator())
            .withEndAction {
                card.animate()
                    .scaleX(1f)
                    .scaleY(1f)
                    .setDuration(120)
                    .start()
            }
            .start()

        val vibrator = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            vibrator.vibrate(VibrationEffect.createOneShot(45, VibrationEffect.DEFAULT_AMPLITUDE))
        } else {
            @Suppress("DEPRECATION")
            vibrator.vibrate(45)
        }
    }

    private fun removeOverlay() {
        overlayView?.let {
            windowManager.removeView(it)
        }
        overlayView = null
        layoutParams = null
    }

    companion object {
        const val ACTION_START = "com.example.whipoverlay.START"
        const val ACTION_STOP = "com.example.whipoverlay.STOP"
        private const val NOTIFICATION_ID = 4001

        @Volatile
        var isRunning: Boolean = false
            private set
    }
}
