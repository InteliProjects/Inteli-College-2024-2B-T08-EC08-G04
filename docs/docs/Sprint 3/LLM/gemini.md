---
title: Gemini
sidebar_label: Gemini
sidebar_position: 2
---

# Documentação da Integração com o GEMINI e IBM Watson

Este documento detalha a integração do sistema com o GEMINI e o IBM Watson, explorando as tecnologias utilizadas, a arquitetura adotada e o fluxo de dados entre os serviços. O objetivo é apresentar uma visão geral da solução implementada, destacando as decisões técnicas e os padrões seguidos.

---

# Introdução

A integração foi desenvolvida para explorar as funcionalidades avançadas do GEMINI, um sistema de análise e visualização de dados, junto com os recursos cognitivos do IBM Watson. O sistema resultante permite interações mais inteligentes e orientadas a dados, proporcionando uma experiência robusta e escalável.

# O que é o Gemini?

O Gemini é uma plataforma de inteligência artificial voltada para a criação de agentes conversacionais avançados. Ele oferece ferramentas para projetar, treinar e implementar chatbots altamente customizáveis, com foco em experiência do usuário e integração com diferentes sistemas.

# Pontos em Comum com o Watson

- **Inteligência Conversacional:** Ambos utilizam IA para processar e interpretar linguagem natural, gerando respostas contextuais e personalizadas.

- **Integração com APIs:** Assim como o Watson, o Gemini permite a criação de APIs para conectar assistentes a outros sistemas.
    
- **Capacidade de Aprendizado Contínuo:** Ambos podem ser treinados continuamente com novos dados para melhorar a precisão das interações.

# Vantagens e Desvantagens do Gemini

**Vantagens:**

- Alta flexibilidade para personalização de fluxos conversacionais.
- Melhor integração com ferramentas analíticas e relatórios detalhados.
- Interface intuitiva para configuração e monitoramento.

**Desvantagens:**

- Dependência de licenciamento específico, que pode aumentar os custos.
- Menor disponibilidade de recursos pré-treinados em comparação com o Watson.
- A curva de aprendizado pode ser maior para iniciantes.


# Tecnologias Utilizadas

## GEMINI

**Descrição:** Plataforma para gerenciamento, análise e visualização de dados.
Uso na Integração:
    - Análise de Dados: Processamento e transformação de dados provenientes de diferentes fontes.
    - Visualização: Geração de gráficos e painéis interativos para suporte à tomada de decisão.
    - Gestão de Recursos: Organização de datasets e metadados para fácil acesso e reutilização.

## IBM Watson Assistant

**Descrição:** Serviço de inteligência artificial para criação de assistentes virtuais.
Uso na Integração:
    - Processamento de entradas do usuário e fornecimento de respostas contextualizadas.
    - Integração com o GEMINI para consultas de dados e geração de insights.

## FastAPI

**Descrição:** Framework web moderno e de alta performance em Python.
Uso na Integração:
    - Criação de endpoints REST para comunicação entre os componentes.
    - Validação de dados utilizando Pydantic.
    - Geração de documentação interativa.

## Pydantic

**Descrição:** Biblioteca para validação de dados e criação de modelos em Python.
Uso na Integração:
    - Definição de schemas para entradas e saídas da API.
    - Garantia de que as requisições enviadas ao GEMINI estão no formato correto.

# Arquitetura da Aplicação

## Estrutura de Pastas e Módulos

A aplicação segue uma arquitetura modular, organizada da seguinte forma:

```
app/
├── __init__.py
├── main.py
├── core/
│   ├── __init__.py
│   └── config.py
├── models/
│   ├── __init__.py
│   └── schemas.py
├── utils/
│   ├── __init__.py
│   ├── watson_assistant.py
│   ├── text_to_speech.py
│   └── audio_player.py
│   └── gemini_api.py
└── api/
    ├── __init__.py
    └── endpoints.py
```

# Fluxo de Dados

1. **Requisição do Cliente**: O cliente faz uma requisição HTTP para a API, enviando uma mensagem de texto.
2. **Endpoint**: A requisição é recebida pelo endpoint correspondente em `api/endpoints.py`.
3. **Validação de Dados**: Os dados de entrada são validados usando os modelos definidos em `models/schemas.py`.
4. **Interação com o Watson Assistant e Gemini**:
   - O módulo `utils/watson_assistant.py` gerencia a sessão e envia a mensagem ao assistente.
   - Quando o cliente envia a mensagem o Gemini produz uma resposta com base na pergunta.
   - Recebe a resposta textual do assistente.
5. **Resposta ao Cliente**: A API retorna ao cliente a resposta do assistente em texto e o áudio codificado em base64. É impotante frizar que ele guarda um histórico de mensgens entre o cliente e o Gemini.

# Descrição dos Módulos (Gemini)

## `utils/gemini_api.py`

- **Função**: Cria as funções de integração com o chatbot do Gemini.
- **Responsabilidades**:
  - função de criar o chat do Gemini (`def start_chatbot`)
  - função de habilitar o envio de mensagens do cliente com o chatbot (`def send_message_to_gemini`)
  - função de deletar o chatbot (`def delete_chatbot`)

## `api/endpoints.py`

- **Função**: Define os endpoints da API.
- **Responsabilidades**:
  - Endpoint para criar sessão (`/session`).
  - Endpoint para enviar mensagem (`/message/{ `session_id` }`).
  - Endpoint para deletar sessão (`/session/{ `session_id` }`).
  - Processamento das requisições e integração entre os módulos.
  - Todos os endpoints faz a integração com o Gemini, quando utiliza o **/session** cria uma sessão e da um start no chatbot, quando utiliza o **/message/{`session_id`}** inicializa as trocas de mensagem com o Gemini, quando utiliza o **/message/{`session_id`}** para deletar a sessão, deleta junto com o chat do Gemini.

# Conclusão

Através da integração do Gemini com o Watson, foi possível criar um chatbot robusto e funcional. O Gemini foi utilizado para gerenciar fluxos conversacionais e personalizar a experiência do usuário, enquanto o Watson adicionou inteligência cognitiva ao interpretar perguntas e fornecer respostas contextuais. Essa combinação permitiu a criação de sessões para que o chatbot interagisse com os clientes de maneira contínua, eficiente e adaptável às necessidades dos usuários. A arquitetura integrada destaca o potencial de unir diferentes plataformas para otimizar soluções em IA conversacional.

# Referências

[1] Inteligência Mil Grau. API do Gemini AI da Google Grátis em Python - Assistente Falante. Disponível em: [https://www.youtube.com/watch?v=bXymjacrklk](https://www.youtube.com/watch?v=bXymjacrklk). Acesso em: 21 novembro. 2024.

[2] GitHub-Inteligência Mil Grau. Chat_Gemini_api.py - Documentação para a criação do chatbot. Disponível em: [https://github.com/inteligenciamilgrau/gemini_api/blob/main/01_gemini_basico_chat.py](https://github.com/inteligenciamilgrau/gemini_api/blob/main/01_gemini_basico_chat.py). Acesso em: 21 novembro. 2024.
