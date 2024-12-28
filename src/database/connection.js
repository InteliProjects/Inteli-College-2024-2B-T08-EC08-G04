const { Client } = require('pg');
require('dotenv').config(); // Para carregar as variáveis do arquivo .env

const client = new Client({
  user: process.env.DB_USER,
  password: process.env.DB_PASSWORD,
});

client.connect()
  .then(() => console.log('Conectado ao banco de dados'))
  .catch(err => console.error('Erro ao conectar ao banco de dados', err.stack));

module.exports = client;
