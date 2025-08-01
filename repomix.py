import subprocess

# 推荐方式：将命令和参数分开写为列表
subprocess.run(["repomix.cmd", "--include", "**/*.py", "--style", "markdown", "."])
