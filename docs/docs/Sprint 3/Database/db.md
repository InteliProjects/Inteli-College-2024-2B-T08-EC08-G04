---
title: Banco de Dados
sidebar_label: Database
sidebar_position: 1
---
# Integração com o Banco de Dados Supabase e API

A integração com o banco de dados é um componente essencial do projeto, pois permite armazenar e acessar dados de forma eficiente, o que é crucial para o desempenho e a escalabilidade do sistema. O banco de dados **Supabase** foi escolhido por ser uma solução escalável, fácil de integrar e altamente compatível com o **PostgreSQL**. A utilização do Supabase no projeto visa não apenas o gerenciamento dos dados dos usuários, mas também a interação com o **LLM** (Language Learning Model), que possui a função de conversar com o idoso, ajudar no acompanhamento de sua saúde mental e fornecer suporte emocional.

A conexão entre o banco de dados e o **LLM** garante que o robô tenha acesso contínuo e em tempo real a dados atualizados sobre a saúde e o estado mental do idoso, possibilitando uma comunicação mais eficiente e personalizada. A seguir, detalharemos o processo de configuração e utilização do banco de dados Supabase e as rotas da API para garantir essa integração.

## Estrutura do Banco de Dados

O banco de dados foi estruturado com o uso de **três tabelas principais**: `usuarios`, `estado_saude_mental`, e `medicos`. Essas tabelas armazenam as informações necessárias para o funcionamento da comunicação entre o robô e o **LLM**, além de garantir que os dados do usuário, seus registros de saúde e médicos responsáveis estejam sempre acessíveis.

### 1. **Tabela `usuarios`**

A tabela `usuarios` contém informações pessoais e dados relacionados à rotina e medicação dos idosos. Essa tabela é fundamental para o LLM, pois fornece informações sobre o perfil do usuário, como os medicamentos que ele usa e a rotina diária.

#### Exemplo de Estrutura da Tabela `usuarios`:

```sql
CREATE TABLE usuarios (
  id SERIAL PRIMARY KEY,
  name VARCHAR(100),
  medications TEXT,
  routine TEXT
);
```

- **`id`**: Identificador único do usuário.
- **`name`**: Nome do usuário (idoso).
- **`medications`**: Medicamentos prescritos ao usuário.
- **`routine`**: Rotina diária do usuário.

### 2. **Tabela `estado_saude_mental`**

A tabela `estado_saude_mental` armazena os registros sobre o estado de saúde mental do idoso. Esse dado é crucial para o LLM, pois ele precisa ajustar o comportamento da interação com base no estado emocional e mental do idoso.

#### Exemplo de Estrutura da Tabela `estado_saude_mental`:

```sql
CREATE TABLE estado_saude_mental (
  id SERIAL PRIMARY KEY,
  usuario_id INT REFERENCES usuarios(id),
  estado VARCHAR(255),
  data TIMESTAMP
);
```

- **`id`**: Identificador único do registro.
- **`usuario_id`**: Chave estrangeira, que faz referência à tabela `usuarios`.
- **`estado`**: Descrição do estado de saúde mental (ex.: "tranquilo", "ansioso", "depressivo").
- **`data`**: Data e hora do registro.

### 3. **Tabela `medicos`**

A tabela `medicos` armazena os dados dos médicos responsáveis pelo acompanhamento dos usuários. O LLM pode usar essas informações para consultar e recomendar o contato com o médico em casos emergenciais ou quando o estado do usuário indicar necessidade de acompanhamento profissional.

#### Exemplo de Estrutura da Tabela `medicos`:

```sql
CREATE TABLE medicos (
  id SERIAL PRIMARY KEY,
  name VARCHAR(100),
  especialidade VARCHAR(100),
  usuario_id INT REFERENCES usuarios(id)
);
```

- **`id`**: Identificador único do médico.
- **`name`**: Nome do médico.
- **`especialidade`**: Especialidade do médico (ex.: clínico geral, psiquiatra).
- **`usuario_id`**: Chave estrangeira, que faz referência à tabela `usuarios`.

## Lógica da Conexão com o Supabase

O cliente Supabase é configurado no arquivo `supabaseClient.js` e é responsável por realizar as consultas ao banco de dados. A conexão é feita usando as variáveis de ambiente, garantindo a segurança e flexibilidade na configuração.

### 1. **Conexão com o Supabase**

A conexão com o Supabase é feita usando a função `createClient` do pacote `@supabase/supabase-js`. O código abaixo demonstra como as credenciais são lidas do arquivo `.env` e a conexão é estabelecida:

```javascript
// src/services/supabaseClient.js
const { createClient } = require('@supabase/supabase-js');
require('dotenv').config();  

const supabaseUrl = process.env.SUPABASE_URL;
const supabaseKey = process.env.SUPABASE_KEY;

const supabase = createClient(supabaseUrl, supabaseKey);  // Criação do cliente Supabase

module.exports = supabase;
```

**Explicação**:
- **`supabaseUrl`** e **`supabaseKey`**: São lidas do arquivo `.env` e usadas para criar o cliente do Supabase.
- **`createClient(supabaseUrl, supabaseKey)`**: Cria a conexão com o banco de dados.

### 2. **Consultas ao Banco de Dados**

O código para fazer consultas ao banco de dados foi organizado em funções específicas, como `selectUsuarios`, `selectEstadoSaudeMental` e `selectMedicos`, que são responsáveis por buscar os dados necessários.

#### Exemplo de Função para Consultar Usuários

```javascript
async function selectUsuarios() {
  try {
    const { data, error } = await supabase
      .from('usuarios')
      .select('*');  // Seleciona todos os registros da tabela `usuarios`

    if (error) {
      throw error;
    }

    return data;  // Retorna os dados encontrados
  } catch (error) {
    console.error('Erro ao consultar a tabela usuarios:', error);
    return null;
  }
}
```

**Explicação**:
- **`supabase.from('usuarios').select('*')`**: Realiza uma consulta à tabela `usuarios` para selecionar todos os registros.
- **Tratamento de erros**: Caso a consulta falhe, o erro será capturado e exibido no console.

#### 3. **Uso do Middleware de Autenticação**

Para garantir que apenas usuários autorizados possam acessar certas rotas, foi implementado um **middleware de autenticação**, que verifica se um **token JWT** válido foi fornecido nas requisições.

```javascript
// src/middlewares/authMiddleware.js
const jwt = require('jsonwebtoken');

function authMiddleware(req, res, next) {
  const token = req.headers['authorization'];

  if (!token) {
    return res.status(403).json({ message: 'Token necessário para acessar esta rota' });
  }

  const tokenWithoutBearer = token.replace('Bearer ', '');

  jwt.verify(tokenWithoutBearer, process.env.JWT_SECRET, (err, user) => {
    if (err) {
      return res.status(403).json({ message: 'Token inválido' });
    }

    req.user = user;
    next();
  });
}

module.exports = authMiddleware;
```

**Explicação**:
- **`jwt.verify(token, process.env.JWT_SECRET)`**: Verifica se o token é válido usando a chave secreta armazenada em **`JWT_SECRET`**.
- **`req.user = user`**: Se o token for válido, ele adiciona as informações do usuário à requisição para ser usado nas próximas etapas.

### 4. **Proteção de Rotas**

Para garantir que as rotas que exigem autenticação não sejam acessadas por usuários não autorizados, o middleware `authMiddleware` é usado em rotas específicas. Aqui está um exemplo de como as rotas são configuradas para exigir um token:

```javascript
// src/routes/userRoutes.js
const express = require('express');
const router = express.Router();
const authMiddleware = require('../middlewares/authMiddleware');
const { getUsuarios } = require('../controllers/userController');

// Rota protegida por autenticação
router.get('/usuarios', authMiddleware, getUsuarios);

module.exports = router;
```

**Explicação**:
- **`authMiddleware`**: A rota `/usuarios` está protegida, e o token JWT é necessário para acessar essa rota.

## Testando a API

Após configurar a API e garantir que as rotas estão protegidas por autenticação, você pode testar as rotas usando o **Postman** ou diretamente no navegador, fornecendo o token JWT no cabeçalho **`Authorization: Bearer <token>`**.

### Conclusão

A integração do Supabase com o projeto garante que o banco de dados esteja bem estruturado e acessível, permitindo que o **LLM** converse de forma inteligente com o idoso. A autenticação com JWT oferece uma camada extra de segurança, protegendo as rotas da API contra acessos não autorizados. O sistema está agora configurado para armazenar e acessar dados em tempo real, garantindo a comunicação eficaz entre o robô e os dados dos usuários.
