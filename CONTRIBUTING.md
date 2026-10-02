# 协作与提交规范

## 代码风格

- C++代码使用仓库根目录的 `.clang-format` 格式化。
- 文件夹、源文件、头文件、变量、函数和命名空间使用小写加下划线。
- 类名使用 CamelCase；私有成员变量使用小写加下划线并以 `_` 结尾。
- 编译期常量使用大写加下划线，并优先使用 `constexpr`。
- 头文件使用 `.hpp` 后缀和 include guard。
- 不使用 `using namespace ...`。
- 物理量变量名应体现单位，例如 `speed_rpm`、`angle_deg`、`yaw_angle_rad`。

## 功能开发流程

1. 每次只实现一个可独立验证的功能。
2. 本地编译并完成对应测试。
3. 记录测试结果和已知问题。
4. 经负责人确认后，再进行 Git 暂存、提交和推送。
5. 每个功能单独形成一次 GitHub 提交，不把多个未验证功能合并提交。

## Commit 前缀

提交信息使用以下格式：

```text
<prefix>: <short description>
```

- `feat`: 新增功能
- `fix`: 修复问题
- `refactor`: 重构，不新增功能、不修复问题
- `docs`: 文档修改
- `style`: 不影响逻辑的格式调整
- `test`: 新增或修改测试
- `chore`: 杂项维护
- `perf`: 性能优化
- `build`: 构建或依赖修改
- `ci`: CI/CD配置修改
- `revert`: 回滚提交

示例：

```text
feat: add dt7 remote control input
fix: correct motor encoder direction
test: verify dual motor linkage
docs: update linkage test procedure
```

## Git操作边界

未经负责人明确确认，不执行以下操作：

- `git add`
- `git commit`
- `git push`
- 创建、切换、合并或删除分支
- 回退或重写提交历史
