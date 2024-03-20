# python3, textual环境部署
echo "Enploying PowerRune Operator environment..."
# 1. 检查python3环境，包括python3和pip3
version=$(python3 -V 2>&1 | awk '{print $2}')
version_pip=$(pip3 -V 2>&1 | awk '{print $2}')

if [ -z "$version" ]; then
    echo "Python3 is not installed, please install it first."
    exit 1
fi
if [ -z "$version_pip" ]; then
    echo "Pip3 is broken, please fix it first."
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