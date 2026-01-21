## 目录结构树

<pre>
Elite_Robots_CS_Interface根目录
├─ include                 # 自有头文件
│  ├─ EliteCSRobot.h
│  ├─ RobotInterface.h
│  └─ RobotGlobal.h
├─ source                  # 自有源文件
│  ├─ EliteCSRobot.cpp
│  └─ test.cpp
├─ thirdparty
│  └─ EliteSDK
│     ├─ include           # 第三方头文件（各平台通用）
│     └─ bin
│        ├─ win-x64        # ← 仅 Windows
│        │  ├─ Debug
│        │  │  ├─ elite-cs-series-sdk.dll
│        │  │  ├─ elite-cs-series-sdk.lib
│        │  │  └─ elite-cs-series-sdk.pdb
│        │  └─ Release
│        │     ├─ elite-cs-series-sdk.dll
│        │     ├─ elite-cs-series-sdk.lib
│        │     └─ elite-cs-series-sdk.pdb
│        └─ linux-x64      # ← 仅 Linux
│           ├─ Debug
│           │  ├─ libelite-cs-series-sdk.a
│           │  └─ libelite-cs-series-sdk.so
│           └─ Release
│              ├─ libelite-cs-series-sdk.a
│              └─ libelite-cs-series-sdk.so
├─ build.bat               # ← 仅 Windows
├─ build.sh                # ← 仅 Linux
├─ CMakeLists.txt
└─ README.md
</pre>

## 构建命令

### Linux
```bash
chmod +x build.sh
./build.sh        # 默认 Release
./build.sh Debug  # 编 Debug
```

### windows
```batch
build.bat          :: 默认 2022 + Release
build.bat 2019     :: 2019 + Release
build.bat 2022 Debug
build.bat 2019 Debug
```

