const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const mongoose = require('mongoose');
const path = require('path');
const authRoutes = require('./routes/auth');
const setupRosConnection = require('./ros/rosConnection');  

const app = express();
const server = http.createServer(app);
const io = socketIo(server);

// MongoDB'ye Bağlanma
mongoose.connect('mongodb://localhost/gokmendatabase', {
  useNewUrlParser: true,
  useUnifiedTopology: true,
});

// Middleware Tanımlama
app.use(express.json());  
app.use('/auth', authRoutes);

setupRosConnection(io);

// Statik Dosyaları Sunma
app.use(express.static('/home/fatih/catkin_ws/src/iq_gnc/scripts/web/GOKMEN_Web/gokmen-app/build'));

// Tüm Diğer Rotalar
app.get('*', (req, res) => {
  res.sendFile(path.resolve('/home/fatih/catkin_ws/src/iq_gnc/scripts/web/GOKMEN_Web/gokmen-app/build', 'index.html'));
});

// Sunucuyu Başlatma
server.listen(3000, () => {
  console.log('Server is listening on port 3000');
});
