#!/bin/bash

# MongoDB servisini başlat (sudoers dosyasını düzenleyerek şifresiz sudo kullanımı sağlanmalı)
echo "Başlatılıyor: MongoDB servisi"
nohup sudo systemctl start mongod &

# MongoDB shell başlat ve 'gokmendatabase' seç
echo "Başlatılıyor: MongoDB shell ve 'gokmendatabase' seçiliyor"
nohup mongosh --eval "use gokmendatabase" &

# Node.js sunucusunu başlat
echo "Başlatılıyor: Node.js sunucusu"
cd ~/catkin_ws/src/iq_gnc/scripts/Gokmen/gokmen-app/src/backend
nohup node server.js &

# Frontend uygulamasını başlat ve tüm sorulara 'yes' cevabı ver
echo "Başlatılıyor: Frontend uygulaması"
cd ~/catkin_ws/src/iq_gnc/scripts/Gokmen/gokmen-app/src
nohup yes | npm start &

# ROS rosbridge websocket başlat (en sona taşıdım)
echo "Başlatılıyor: ROS rosbridge websocket"
nohup roslaunch rosbridge_server rosbridge_websocket.launch &

# Sona 10 saniye gecikme ekleyin
sleep 10

echo "Tüm servisler başlatıldı."

# Betiği sonsuza kadar çalıştır
wait

