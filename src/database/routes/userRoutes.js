const express = require('express');
const router = express.Router();
const { getUsuarios, getEstadoSaudeMental, getMedicos } = require('../controllers/userController');
const { selectUsuarios, selectEstadoSaudeMental, selectMedicos } = require('../services/supabaseClient');


router.get('/usuarios/:id', async (req, res) => {
  try {
    const usuarioId = req.params.id;
    const usuarios = await selectUsuarios();
    const usuario = usuarios.find(user => user.id == usuarioId);

    if (!usuario) {
      return res.status(404).json({ message: 'Usuário não encontrado' });
    }

    res.json(usuario); 
  } catch (error) {
    console.error('Erro ao consultar o banco de dados:', error);
    res.status(500).json({ message: 'Erro ao consultar o banco de dados' });
  }
});

// Rota para pegar todos usuarios

router.get('/usuarios', async (req, res) => {
  try {
    const usuarios = await selectUsuarios();
    res.json(usuarios);
  } catch (error) {
    console.error('Erro ao consultar o banco de dados:', error);
    res.status(500).json({ message: 'Erro ao consultar o banco de dados' });
  }
}
);

router.get('/saude-mental/:id', async (req, res) => {
  try {
    const mentalId = req.params.id;
    const estados = await selectEstadoSaudeMental();
    const estado = estados.find(est => est.user_id == mentalId);

    if (!estado) {
      return res.status(404).json({ message: 'Dados de saúde mental não encontrados' });
    }

    res.json(estado); 
  } catch (error) {
    console.error('Erro ao consultar a tabela estado_saude_mental:', error);
    res.status(500).json({ message: 'Erro ao consultar o banco de dados' });
  }
});



router.get('/usuarios/:id', async (req, res) => {
    try {
      const usuarioId = req.params.id;
      const usuarios = await selectUsuarios();
      const usuario = usuarios.find(user => user.id == usuarioId);
  
      if (!usuario) {
        return res.status(404).json({ message: 'Usuário não encontrado' });
      }
  
      res.json(usuario); 
    } catch (error) {
      console.error('Erro ao consultar o banco de dados:', error);
      res.status(500).json({ message: 'Erro ao consultar o banco de dados' });
    }
  });
  
  router.get('/saude-mental/:id', async (req, res) => {
    try {
      const mentalId = req.params.id;
      const estados = await selectEstadoSaudeMental();
      const estado = estados.find(est => est.user_id == mentalId);
  
      if (!estado) {
        return res.status(404).json({ message: 'Dados de saúde mental não encontrados' });
      }
  
      res.json(estado); 
    } catch (error) {
      console.error('Erro ao consultar a tabela estado_saude_mental:', error);
      res.status(500).json({ message: 'Erro ao consultar o banco de dados' });
    }
  });
  

module.exports = router;
