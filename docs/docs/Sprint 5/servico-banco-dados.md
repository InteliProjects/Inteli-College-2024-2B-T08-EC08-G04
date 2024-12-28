---
title: Atualização do Banco de Dados
sidebar_label: Database Updates
sidebar_position: 2
---

# Atualização na Integração com o Banco de Dados - Sprint 5

Essa sessão descreve as atualizações feitas no sistema de integração com o banco de dados durante a Sprint 5. Estas atualizações eliminam a necessidade do uso de tokens JWT para autenticação, simplificando o fluxo de comunicação e melhorando a acessibilidade das rotas de dados. Esta implementação se baseia na documentação da Sprint 3 para manter consistência e continuidade.

## Contexto das Atualizações

Na Sprint 3, conforme documentado, o sistema utilizava tokens JWT para autenticação, garantindo que apenas usuários autorizados pudessem acessar as rotas protegidas. Agora, durante a Sprint 5, as seguintes mudanças foram implementadas:

1. **Eliminação do Middleware de Autenticação**:
   - Não há mais necessidade de verificar tokens JWT para cada requisição.
   - Melhorias no desempenho devido à redução de sobrecarga no processamento de tokens.

2. **Simplificação das Rotas de API**:
   - Novas rotas foram implementadas para acessar diretamente os dados do banco de dados.
   - Consultas otimizadas foram adicionadas para facilitar a extração de informações do banco.

## Novas Rotas de API

### 1. Rota para Obter Informações de Usuário

```javascript
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
```

### 2. Rota para Obter Dados de Saúde Mental

```javascript
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
```

### 3. Rota para Obter Informações Completas de Usuário

```javascript
router.get('/usuario-completo/:id', async (req, res) => {
    try {
        const usuarioId = req.params.id;
        const usuarios = await selectUsuarios();
        const estados = await selectEstadoSaudeMental();

        const usuario = usuarios.find(user => user.id == usuarioId);
        const estado = estados.find(est => est.user_id == usuarioId);

        if (!usuario || !estado) {
            return res.status(404).json({ message: 'Dados do usuário ou de saúde mental não encontrados' });
        }

        res.json({
            usuario,
            estado
        });
    } catch (error) {
        console.error('Erro ao consultar os dados completos do usuário:', error);
        res.status(500).json({ message: 'Erro ao consultar o banco de dados' });
    }
});
```

## Testando as Novas Rotas

Com as novas rotas implementadas, a API pode ser testada diretamente utilizando ferramentas como **Postman** ou via solicitações HTTP simples. Exemplos de endpoints disponíveis:

- **Obter informações de um usuário**: `GET /usuarios/:id`
- **Obter dados de saúde mental de um usuário**: `GET /saude-mental/:id`
- **Obter informações completas de um usuário**: `GET /usuario-completo/:id`

## Considerações Finais

Esta atualização simplifica o fluxo de integração do sistema com o banco de dados, melhorando o desempenho e a manutenção do código. Além disso, a ausência de autenticação baseada em JWT reduz a complexidade da API e melhora a experiência do desenvolvedor.
