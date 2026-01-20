## 目录结构树

<pre>
Elite_Robots_CS_Interface根目录
├─ include          # 自有头文件
│  ├─ EliteCSRobot.h
│  ├─ RobotInterface.h
│  └─ RobotGlobal.h
├─ source           # 自有源文件
│  ├─ EliteCSRobot.cpp
│  └─ test.cpp
├─ thirdparty
│  └─ EliteSDK
│     ├─ bin        # 动态库 .dll/.so
│     │  ├─ libelite-cs-series-sdk.a
│     │  └─ libelite-cs-series-sdk.so
│     └─ include    # 第三方头文件
├─ build            # 推荐 out-of-source 编译
├─ CMakeLists.txt
└─ README.md
</pre>

## 构建命令

chmod +x build.sh

./build.sh
