const { selectUsuarios, selectEstadoSaudeMental, selectMedicos } = require('../services/supabaseClient');


async function getUsuarios(req, res) {
  try {
    const usuarios = await selectUsuarios();
    if (usuarios) {
      res.status(200).json(usuarios);  // Retorna os dados dos usuários
    } else {
      res.status(404).json({ message: 'Nenhum usuário encontrado' });
    }
  } catch (error) {
    res.status(500).json({ message: 'Erro ao obter usuários', error });
  }
}


async function getEstadoSaudeMental(req, res) {
  try {
    const estado = await selectEstadoSaudeMental();
    if (estado) {
      res.status(200).json(estado);  // Retorna os dados de estado de saúde mental
    } else {
      res.status(404).json({ message: 'Nenhum dado de saúde mental encontrado' });
    }
  } catch (error) {
    res.status(500).json({ message: 'Erro ao obter dados de saúde mental', error });
  }
}


async function getMedicos(req, res) {
  try {
    const medicos = await selectMedicos();
    if (medicos) {
      res.status(200).json(medicos);  // Retorna os dados dos médicos
    } else {
      res.status(404).json({ message: 'Nenhum médico encontrado' });
    }
  } catch (error) {
    res.status(500).json({ message: 'Erro ao obter médicos', error });
  }
}

module.exports = { getUsuarios, getEstadoSaudeMental, getMedicos };
