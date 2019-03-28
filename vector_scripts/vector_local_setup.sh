# Instalation of COSKMO SDK Localy
mkdir Vector_Playground
cd Vector_Playground

# Create and activate your own VEnv to not mess up your local instalation UBUNTU 16
sudo apt-get update
sudo apt-get install python-virtualenv
virtualenv -p /usr/bin/python3.6 vector_env 
source vector_env/bin/activate

# INstall SDK
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt-get update
sudo apt-get install python3.6
sudo apt install python3-pip
sudo apt-get install build-essential libssl-dev libffi-dev python3.6-dev
python3.6 -m pip install anki_vector

# You need the Name, ID and IP all explained how to get them here: https://developer.anki.com/vector/docs/troubleshooting.html#can-t-find-vector-s-ip-address
export PYTHONPATH=/usr/local/lib/python3.6/site-packages:$PYTHONPATH
python3 -m anki_vector.configure

# For the demo of 3d View
pip3 install "anki_vector[3dviewer]"

# Download the Tutorials
wget https://sdk-resources.anki.com/vector/0.5.1/anki_vector_sdk_examples_0.5.1.tar.gz
tar xvzf anki_vector_sdk_examples_0.5.1.tar.gz
cd anki_vector_sdk_examples_0.5.1/tutorials

# To force your system to look for the correct opencv version, we add to the fron the path where your opencv-python for python3.6 is


# VECTOR should say :Hello World
./01_hellow_world.py




