# Elite Robots CS Interface

跨平台 C++ 封装库，用于连接与控制 Elite 机器人 CS 系列，内置 GoogleTest 单元测试。

---

## 目录结构

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
│  ├─ EliteSDK             # SDK依赖库文件
│  │  ├─ include           # 第三方头文件
│  │  └─ bin               # 预编译二进制(dll/lib/so)
│  └─ gtest                # GTest源码子模块（空目录→git submodule）
├─ build.bat               # Windows构建脚本
├─ build.sh                # Linux构建脚本
├─ CMakeLists.txt
└─ README.md
</pre>

---

## 快速开始

### 1. 克隆（含子模块）

```bash
git clone --recurse-submodules https://github.com/your_name/Elite_Robots_CS_Interface.git
cd Elite_Robots_CS_Interface
```

> 若已克隆但未带子模块，执行：
> ```bash
> git submodule update --init --recursive
> ```

### 2. 一键构建

**Windows**（推荐 **x64 Native Tools Command Prompt**）：
```powershell
# 仅编译核心库（Release）
build.bat

# 编译核心库 + 单元测试（Debug）
build.bat Debug test
```

**Linux**：
```bash
chmod +x build.sh
# 编译核心库 + 单元测试（Debug）
./build.sh Debug test
```

### 3. 运行测试
**注意**：`ctest -C` 必须与**构建配置**一致！

**Windows**（**Release** 示例）：
```powershell
ctest --test-dir build\win-Release -C Release --output-on-failure
# Debug 时：
#   ctest --test-dir build\win-Debug -C Debug --output-on-failure
```

**Linux**（**Release** 示例）：
```bash
ctest --test-dir build/linux-Release --output-on-failure
# Debug 时：
#   ctest --test-dir build/linux-Debug --output-on-failure
```

---

## 构建选项

| 参数 | 说明 |
| --- | --- |
| `Release` / `Debug` | 选择构建配置 |
| `test` | 同时编译 GoogleTest 单元测试 |

---

## 依赖说明

- **CMake ≥ 3.16**
- **C++17 编译器**
- **操作系统**：Windows 10/11（VS2019/2022）或主流 Linux 发行版
- **第三方库**：
  - EliteSDK（已预置）
  - GoogleTest（源码子模块，无需系统安装）

---

## 许可证

[在此处填写你的许可证信息]
