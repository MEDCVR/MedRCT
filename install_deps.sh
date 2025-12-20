sudo apt-get update
sudo apt install -y libspdlog-dev libeigen3-dev liburdfdom-dev libfmt-dev libyaml-cpp-dev curl unzip
curl -s https://packagecloud.io/install/repositories/dirk-thomas/colcon/script.deb.sh | sudo bash
sudo apt install -y python3-colcon-common-extensions
pip3 install pygame
