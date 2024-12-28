const express = require('express');
const app = express();
const userRoutes = require('./routes/userRoutes'); 
const authMiddleware = require('./middlewares/authMiddleware')

// Middleware para processar JSON no corpo das requisições
app.use(express.json());

// app.use(authMiddleware);

app.use('/api', userRoutes);

app.use((req, res, next) => {
  res.status(404).json({ message: 'Rota não encontrada' });
});

module.exports = app; // Exporta o app para ser usado no server.js
