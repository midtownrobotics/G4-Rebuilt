# Pointing VS Code at your WPILib JDK

`.vscode/settings.json` is committed (Gradle/IntelliSense settings everyone needs),
so it must not contain machine-specific absolute paths. The WPILib JDK lives in a
different place on each OS:

| OS          | WPILib 2026 JDK path                  |
| ----------- | ------------------------------------- |
| Windows     | `C:\Users\Public\wpilib\2026\jdk`     |
| macOS/Linux | `~/wpilib/2026/jdk` (e.g. `/Users/you/wpilib/2026/jdk`) |

Set it **once per machine** in your **User** settings, which are not in git:

1. `Cmd/Ctrl+Shift+P` -> **Preferences: Open User Settings (JSON)**
2. Add (using your own OS's path from the table above):

```jsonc
"java.jdt.ls.java.home": "/Users/you/wpilib/2026/jdk",
"java.import.gradle.java.home": "/Users/you/wpilib/2026/jdk"
```

3. `Cmd/Ctrl+Shift+P` -> **Java: Clean Java Language Server Workspace** -> Restart.

If you hit the error "The java.jdt.ls.java.home variable ... points to a missing or
inaccessible folder", someone re-added the path to `.vscode/settings.json` — remove
it there (workspace settings override user settings) rather than changing it to your
own path.
