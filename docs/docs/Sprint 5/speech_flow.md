---
title: Fluxo da Integração
sidebar_label: Fluxo da Integração
sidebar_position: 2
---

# Introdução

&emsp;&emsp;Nesta seção será abordada a integração, especialmente da tela de comunicação, descrevendo cada parte do sistema que é ativada e a ordem de ativação para realizar a comunicação. Para uma melhor visualização do fluxo de funcionamento da conversação, foi criado um diagrama que pode ser observado na seção abaixo:

# Fluxo da Conversa

<p align="center"> Figura 1 - Diagrama</p>
<div align="center" class="zoom-image">
  <img src={require('../../static/img/sprint-5/conversation_flow.png').default} alt="10 falhas"/>
</div>
<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe J.A.R.B.A.S.</p>

## Recebimento do áudio

&emsp;&emsp;Como pode ser observado acima, o frontend capta a informação do usuário através do botão de fala, convertendo a fala do usuário em um arquivo de áudio e enviando-o ao primeiro serviço, o STT (Speech to Text) da IBM Cloud. Esse serviço recebe o arquivo de áudio e o transforma em uma string (texto), a fim de ser utilizada em um chatbot.

## Escolha do Chatbot

&emsp;&emsp;Com o input (string de texto), o conteúdo é enviado primeiramente ao Rasa, que identifica se a fala está relacionada às funções pré-definidas, que são: onboarding, emergência, check-up diário e movimentação. Caso o Rasa não identifique que a fala esteja correlacionada às funções pré-definidas, o mesmo input é enviado ao Gemini para obter uma resposta mais geral e usual para o usuário.

&emsp;&emsp;Vale notar que tanto o Rasa quanto o Gemini recebem o contexto da conversa e o estado do usuário antes de interagir com ele. Sendo assim, as respostas não estarão tão distantes do contexto do usuário, permitindo manter uma relação de conversa contínua.

## Envio do áudio

&emsp;&emsp;Após a resposta ser gerada, ela é enviada a outro serviço da IBM – o TTS (Text to Speech) – que converte a string de resposta do chatbot em um arquivo de áudio. Esse arquivo é enviado ao frontend juntamente com a string de resposta, que aparece na tela junto com a fala que é reproduzida em sincronia com o texto.   