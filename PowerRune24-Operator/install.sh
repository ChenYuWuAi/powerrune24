# python3, textual环境部署
echo "Enploying PowerRune Operator environment..."
# 1. 检查python3环境，包括python3和pip3，python3版本必须大于3.7
version=$(python3 -V 2>&1 | grep -Po '(?<=Python )(\d+\.\d+)' || echo "")
version_pip=$(pip3 -V 2>&1 | grep -Po '(?<=python )(\d+\.\d+)' || echo "")

if [ -z "$version" ]; then
    echo "Python3 is not installed."
    exit 1
fi

if [ $(echo "$version < 3.7" | bc) -eq 1 ]; then
    echo "Python3 version must be greater than 3.7."
    exit 1
fi

if [ -z "$version_pip" ]; then
    echo "Pip3 is not installed."
    exit 1
fi

echo "Python3 version: $version"
# 2. 安装依赖
echo "Installing requirements..."

if ! pip3 install -r requirements.txt; then
    echo "Failed to install requirements, please check your network and try again."
    exit 2
fi

# 3. 安装完成
echo "Install finished. \nPowerRune Operator environment is ready."
exit 0