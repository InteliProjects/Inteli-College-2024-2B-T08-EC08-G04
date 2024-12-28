---
title: Interface STT/TTS
sidebar_label: Interface STT/TTS
sidebar_position: 1
---

# Interface STT/TTS

Entre as funcionalidades principais do sistema desenvolvido para a solução J.A.R.B.A.S., está a conversação com o robô autônomo, que deve acontecer por meio de uma interface gráfica. Nessa interface, o usuário deve ser capaz de se comunicar com o robô por comandos de voz, que são convertidos para prompts em texto (o que define o serviço de Speech To Text). Com esses prompts, o serviço de LLM existente na solução gera uma resposta em texto que é convertida para um áudio reproduzido para o usuário (o que define o serviço de Text To Speech).

Para isso, a equipe J.A.R.B.A.S. criou uma página na interface mobile na qual o usuário idoso pode se utilizar dessa funcionalidade.

<p align="center"> Figura 1 - Tela de conversação (botão não pressionado) </p>
<div align="center" class="zoom-image">
  <img src={require('../../static/img/sprint-4/conversacao1.png').default} alt="Tela de conversação antes do usuário pressionar o botão"
  style={{height: '420px'}} />
</div>
<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe J.A.R.B.A.S.</p>

Essa tela, existente previamente na sprint 3, foi aprimorada de forma que os serviços de TTS e STT foram totalmente integrados a ela e sua estrutura e estilização foram adequadas à WCAG, além de apresentar uma variação exibida especificamente nos momentos de resposta do serviço de LLM. O processo existente preliminarmente continua o mesmo:

1. O usuário pressiona e segura o botão central com ícone de microfone na tela
2. Enquanto o botão está pressionado, o usuário pode falar o que deseja comunicar ao robô
3. Após terminar sua fala, o usuário solta o botão e deve aguardar o processo se STT ocorrer
4. Com o processo STT concluído, o serviço de LLM gera uma resposta para a fala do usuário
5. Essa resposta é exibida tanto em formato de texto quanto em formato de áudio (através do processo de TTS) para o usuário

A principal mudança feita na tela de conversação entre a sprint 3 e a sprint 4, então, está na forma como a respota do LLM é exibida para o usuário, como sugere a figura 2.

<p align="center"> Figura 2 - Tela de conversação (após pressionamento do botão) </p>
<div align="center" class="zoom-image">
  <img src={require('../../static/img/sprint-4/conversacao2.png').default} alt="Tela de conversação após o usuário pressionar o botão"
   style={{height: '420px'}} />
</div>
<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe J.A.R.B.A.S.</p>

Agora, não há mais a transcrição e a opção de playback do áudio gravado do usuário, já que a exibição de ambos não é interessante para este. Em vez disso, a tela de conversção apresenta uma nova estrutura no momento de resposta, com o botão de microfone se tornando um ícone indicativo de áudio e uma caixa de texto surgindo logo abaixo dele com a transcrição da resposta. Dessa forma, o usuário consegue ler e ouvir a resposta do LLM de maneira muito mais rápida e eficiente.