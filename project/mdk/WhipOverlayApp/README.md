# WhipOverlayApp

An Android floating companion app inspired by the "tap to hurry up" joke workflow.

This first MVP does three things:

1. Requests overlay permission.
2. Starts a foreground service with a draggable floating bubble.
3. Triggers haptic and visual feedback when you tap the bubble.

## What this version does not do yet

- It does not control the Doubao app UI.
- It does not inject text into another app.
- It does not use accessibility services.

That is intentional for the first cut: the overlay interaction is stable, easy to test, and much easier to share with other people.

## Project layout

- `app/src/main/java/com/example/whipoverlay/MainActivity.kt`
- `app/src/main/java/com/example/whipoverlay/OverlayService.kt`
- `app/src/main/java/com/example/whipoverlay/NotificationHelper.kt`

## Open in Android Studio

1. Open the `WhipOverlayApp` folder in Android Studio.
2. Let Android Studio install the Android SDK / Gradle components it asks for.
3. Use an Android 8.0+ phone or emulator.
4. Run the `app` configuration.

## MVP flow

1. Open the app.
2. Grant "draw over other apps" permission.
3. Tap `Start overlay`.
4. Move to Doubao or any other app.
5. Drag the floating bubble anywhere you like.
6. Tap the bubble to trigger a "whip" animation, vibration, and status message.

## Next good upgrades

- Add custom sound effects.
- Add skins and phrase packs.
- Add a compact settings panel in the overlay.
- Add optional clipboard copy for a preset "speed up" prompt.
- Add accessibility automation only if you explicitly want app-to-app UI interaction.
