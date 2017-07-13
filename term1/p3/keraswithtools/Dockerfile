FROM dulemba/tfcv2
RUN apt-get update && apt-get -y -o Dpkg::Options::="--force-confmiss" install --reinstall netbase
RUN pip3 install h5py python-socketio eventlet flask Pillow keras
