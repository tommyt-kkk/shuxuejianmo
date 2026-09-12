# 李忱昊到此一游
# B题论文可编辑 LaTeX 工程

## 最简单的使用方法

1. 用 VS Code 打开整个 `B题论文_可编辑LaTeX工程_20260912` 文件夹，不要只打开单个文件。
2. 打开 `main.tex`，或打开 `sections` 文件夹中的相应章节。
3. 修改文字后按 `Ctrl+S` 保存。
4. LaTeX Workshop 会自动使用 XeLaTeX 连续编译两遍。
5. 生成的预览文件位于 `.latex-build/main.pdf`。

如果保存后没有自动编译，可按 `Ctrl+Alt+B`；也可以按 `Ctrl+Shift+P`，运行 `LaTeX Workshop: Build LaTeX project`。

## 文件对应关系

- `main.tex`：论文总入口、摘要、章节顺序、AI 工具使用声明与附录入口。
- `sections/`：正文各章节和附录文件，平时主要修改这里。
- `references.tex`：参考文献。
- `figures/`：论文图片。
- `appendix_code/`：附录中的源程序代码。
- `.vscode/settings.json`：本工程的自动编译设置。
- `.latex-build/`：编译生成的 PDF 和缓存文件，不要在这里修改正文。

## 修改时注意

- `%`、`_`、`&`、`#`、`$` 在 LaTeX 中有特殊含义；作为普通文字使用时，通常应写成 `\%`、`\_`、`\&`、`\#`、`\$`。
- 不要删除 `\begin{...}`、`\end{...}`、花括号或 `\input{...}` 等排版命令。
- 摘要须保持一页；正文不要恢复目录；AI 工具使用声明应位于参考文献之前；附录位于参考文献之后。
- 每次提交前，把 `.latex-build/main.pdf` 复制到项目根目录的 `输出` 文件夹并重新生成 MD5；生成 MD5 后不要再改文件。
