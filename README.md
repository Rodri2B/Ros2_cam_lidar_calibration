# Ros2 Camera-Lidar Simulator and Calibrator

Neste repositório se encontram os códigos desenvolvidos para a simulação e calibração para câmera e lidar. Além disso, também constam os nós utilizados no ROS para fazer o interfaceamento das duas ferramentas com o ecossistema do ROS 2.

A metodologia utilizada foi baseada em duas publicações: ["Automatic extrinsic calibration between a camera and a 3D Lidar using 3D point and plane correspondences"](https://arxiv.org/abs/1904.12433) e ["Optimising the selection of samples for robust lidar camera calibration"](https://arxiv.org/abs/2103.12287). A primeira apresenta a metologia que foi utilizada para a forma de representar as amostras, estabelecer métricas e realizar a calibração de fato. A segunda publição foi de onde foram retiradas as métricas para avaliar a qualidade das amostras selecionadas e também a técnica de separar as amostras em pequenos grupos para designa-las ao algorítimo de calibração de maneira a maximizar a precisão do resultado final da calibração.






