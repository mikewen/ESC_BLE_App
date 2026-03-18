# Nordic BLE Library
-keep class no.nordicsemi.android.ble.** { *; }

# Keep BleManager subclass
-keep class com.esccontroller.AC6328BleManager { *; }

# Kotlin
-keep class kotlin.** { *; }
-keepclassmembers class **$WhenMappings { <fields>; }
