package com.example.whipoverlay

import android.Manifest
import android.content.Intent
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.provider.Settings
import android.widget.Button
import android.widget.TextView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat

class MainActivity : AppCompatActivity() {

    private lateinit var overlayStatusValue: TextView
    private lateinit var serviceStatusValue: TextView
    private lateinit var tipValue: TextView

    private val notificationPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { updateStatus() }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        overlayStatusValue = findViewById(R.id.overlay_status_value)
        serviceStatusValue = findViewById(R.id.service_status_value)
        tipValue = findViewById(R.id.tip_value)

        findViewById<Button>(R.id.request_overlay_button).setOnClickListener {
            openOverlayPermissionScreen()
        }

        findViewById<Button>(R.id.start_overlay_button).setOnClickListener {
            maybeRequestNotificationPermission()
            if (Settings.canDrawOverlays(this)) {
                ContextCompat.startForegroundService(
                    this,
                    Intent(this, OverlayService::class.java).setAction(OverlayService.ACTION_START)
                )
                tipValue.text = getString(R.string.tip_started)
            } else {
                tipValue.text = getString(R.string.tip_need_overlay)
                openOverlayPermissionScreen()
            }
            updateStatus()
        }

        findViewById<Button>(R.id.stop_overlay_button).setOnClickListener {
            stopService(Intent(this, OverlayService::class.java))
            tipValue.text = getString(R.string.tip_stopped)
            updateStatus()
        }
    }

    override fun onResume() {
        super.onResume()
        updateStatus()
    }

    private fun updateStatus() {
        val overlayGranted = Settings.canDrawOverlays(this)
        overlayStatusValue.text = if (overlayGranted) {
            getString(R.string.status_ready)
        } else {
            getString(R.string.status_missing)
        }

        serviceStatusValue.text = if (OverlayService.isRunning) {
            getString(R.string.status_running)
        } else {
            getString(R.string.status_idle)
        }
    }

    private fun openOverlayPermissionScreen() {
        val intent = Intent(
            Settings.ACTION_MANAGE_OVERLAY_PERMISSION,
            Uri.parse("package:$packageName")
        )
        startActivity(intent)
    }

    private fun maybeRequestNotificationPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            if (checkSelfPermission(Manifest.permission.POST_NOTIFICATIONS) !=
                android.content.pm.PackageManager.PERMISSION_GRANTED
            ) {
                notificationPermissionLauncher.launch(Manifest.permission.POST_NOTIFICATIONS)
            }
        }
    }
}
