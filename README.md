## 目录结构树

<pre>
Elite_Robots_CS_Interface根目录
├─ Elite_Robots_CS_SDK     # 原厂SDK子模块(包含文档/示例)
├─ include                 # 接口封装头文件
│  ├─ EliteCSRobot.h
│  ├─ RobotInterface.h
│  └─ RobotGlobal.h
├─ source                  # 接口封装源文件
│  ├─ EliteCSRobot.cpp
│  └─ test.cpp             # 简单集成测试主入口
├─ resources               # 机器人控制脚本与配置文件
│  ├─ external_control.script
│  ├─ input_recipe.txt
│  └─ output_recipe.txt
├─ test                    # 单元/集成测试模块 (GTest)
│  ├─ CMakeLists.txt
│  ├─ ConnectionTest.cpp   # 连接/断开/重连测试
│  ├─ MotionTest.cpp       # 运动指令测试(MoveTo/Linear/Circular/Jog)
│  ├─ StateTest.cpp        # 状态查询与信息获取测试
│  └─ PathTest.cpp         # 路径录制与回放功能测试
├─ thirdparty
│  └─ EliteSDK             # SDK依赖库文件
│     ├─ include           # 第三方头文件
│     └─ bin               # 预编译二进制(dll/lib/so)
├─ build.bat               # Windows构建脚本
├─ build.sh                # Linux构建脚本
├─ CMakeLists.txt
└─ README.md
</pre>

## 构建与测试

本项目采用 CMake 构建，并提供了便捷的批处理脚本。

### Windows

推荐在 **x64 Native Tools Command Prompt for VS 2019/2022** 中执行：

1. **编译**

   ```batch
   # 1. 仅编译核心库 (Release)
   build.bat

   # 2. 仅编译核心库 (Debug)
   build.bat Debug

   # 3. 编译核心库 + 单元测试 (Release/Debug)
   build.bat Release test
   build.bat Debug test
   ```

2. **运行测试**
   （需先执行带 `test` 参数的构建命令）

   ```batch
   # 方式一：使用 ctest (在根目录执行)
   ctest --test-dir build/win-Debug -C Debug --output-on-failure

   # 方式二：直接运行测试程序
   build\win-Debug\test\Debug\ConnectionTest.exe
   ```

### Linux

1. **编译**

   ```bash
   chmod +x build.sh

   # 1. 仅编译核心库 (Release)
   ./build.sh

   # 2. 仅编译核心库 (Debug)
   ./build.sh Debug

   # 3. 编译核心库 + 单元测试
   ./build.sh Release test
   ./build.sh Debug test
   ```

2. **运行测试**
   ```bash
   # 在构建目录运行 ctest
   ctest --test-dir build/linux-Debug --output-on-failure
   ```
