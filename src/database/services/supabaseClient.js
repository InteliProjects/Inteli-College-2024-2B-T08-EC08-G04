const { createClient } = require('@supabase/supabase-js');
require('dotenv').config();


const supabaseUrl = process.env.SUPABASE_URL;
const supabaseKey = process.env.SUPABASE_KEY;

console.log(supabaseKey)
console.log(supabaseUrl)

const supabase = createClient(supabaseUrl, supabaseKey);

async function selectUsuarios() {
  try {
    const { data, error } = await supabase
      .from('usuarios')
      .select('*');  

    if (error) {
      throw error;
    }

    return data;  
  } catch (error) {
    console.error('Erro ao consultar a tabela usuarios:', error);
    return null;
  }
}

async function selectEstadoSaudeMental() {
  try {
    const { data, error } = await supabase
      .from('estado_saude_mental')
      .select('*');  

    if (error) {
      throw error;
    }

    return data;  
  } catch (error) {
    console.error('Erro ao consultar a tabela estado_saude_mental:', error);
    return null;
  }
}

async function selectMedicos() {
  try {
    const { data, error } = await supabase
      .from('medicos')
      .select('*');  

    if (error) {
      throw error;
    }

    return data;  
  } catch (error) {
    console.error('Erro ao consultar a tabela medicos:', error);
    return null;
  }
}

module.exports = { selectUsuarios, selectEstadoSaudeMental, selectMedicos };
