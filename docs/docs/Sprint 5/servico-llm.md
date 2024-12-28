---
title: Serviço LLM
sidebar_label: Serviço LLM
sidebar_position: 5
---

# Serviço LLM do Projeto J.A.R.B.A.S.

O serviço LLM (Large Language Model) do J.A.R.B.A.S. é responsável por intermediar a comunicação entre a entrada de voz do usuário, o processamento de linguagem natural por meio da API do Gemini, e a posterior conversão da resposta em áudio. Este serviço funciona como um “cérebro” conversacional, recebendo comandos de voz, interpretando-os, enviando-os para o modelo de linguagem e retornando a resposta para o usuário, também em formato de voz.

## Funcionalidades Principais

- **Recebimento de áudio (voz):** O serviço recebe o áudio capturado pela interface, seja por meio do robô ou outro dispositivo de captura de voz.
- **Conversão de voz para texto (STT - Speech-to-Text):** Utilizando ferramentas de reconhecimento de fala, o serviço converte o áudio recebido em texto para posterior envio à API do Gemini.
- **Interação com o Modelo de Linguagem (Gemini):** Após a conversão para texto, o serviço envia a solicitação para a API do Gemini, recebendo em troca uma resposta textual coerente e contextualizada.
- **Conversão de texto para voz (TTS - Text-to-Speech):** A resposta em texto obtida do Gemini é convertida novamente em áudio, permitindo ao usuário ouvir a resposta do sistema.
- **Integração com Banco de Dados (PostgreSQL):** O serviço interage com um banco de dados PostgreSQL para armazenamento e recuperação de informações relevantes, sejam logs de conversas, perfis de usuários ou outros dados necessários.
  
## Arquitetura e Fluxo de Dados

Abaixo está a representação simplificada do fluxo de dados:

1. **Entrada de voz:** O usuário fala um comando ou pergunta.
2. **STT:** O áudio é processado pelo serviço para extrair o texto.
3. **Envio à API do Gemini:** O texto resultante é enviado para a API do Gemini, que processa e retorna uma resposta.
4. **Retorno de resposta textual:** O serviço recebe a resposta do Gemini e prepara-a para conversão.
5. **TTS:** A resposta é transformada em áudio.
6. **Saída de voz:** O usuário ouve a resposta final.

Esse fluxo garante uma interação natural com o sistema, permitindo que o usuário utilize linguagem falada para se comunicar com o robô.

## Stack Tecnológica

- **Backend:** Desenvolvido em Python com o framework **FastAPI** para criação de endpoints REST.
- **Banco de Dados:** **PostgreSQL** para armazenamento persistente de informações.
- **API LLM (Gemini):** Serviço externo que fornece processamento de linguagem natural de alta qualidade.
- **STT/TTS:** Integração com ferramentas de reconhecimento e síntese de voz.  
  *(A escolha da ferramenta STT/TTS dependerá da implementação, podendo variar entre soluções como Google Cloud Speech-to-Text, Amazon Polly, ou outras semelhantes.)*

## Estrutura do Projeto

A imagem em anexo mostra a estrutura de diretórios do serviço LLM. Uma possível organização é:

- **backend/**
  - **api_robot/**: Contém lógicas específicas da interação com o robô, possivelmente incluindo endpoints para envio e recebimento de áudio.
  - **llm_service/**: Diretório principal do serviço LLM.
    - **app/**: Diretório contendo o código da aplicação.
      - **api/**: Endpoints FastAPI responsáveis por receber requisições de voz e retornar respostas.
      - **controlers/**: Controladores contendo a lógica de interação entre STT, TTS, base de dados e API do Gemini.
      - **core/**: Arquivos de configuração, inicialização, middlewares e utilitários centrais do serviço.
      - **models/**: Definições de modelos de dados, schemas (pydantic) e classes relacionadas ao banco.
      - **utils/**: Funções utilitárias e auxiliares, como conexão com STT/TTS, formatação de dados, etc.
    - **__init__.py**: Arquivo que permite que o diretório seja tratado como um módulo Python.


## Conclusão

O serviço LLM do J.A.R.B.A.S. integra diversos componentes - reconhecimento de voz, modelo de linguagem, síntese de voz e banco de dados - para proporcionar uma experiência conversacional fluida. Essa camada permite que o usuário interaja por meio de fala com o sistema, recebendo respostas coerentes e úteis em tempo real.
