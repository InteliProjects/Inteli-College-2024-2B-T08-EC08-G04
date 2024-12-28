---
title: Teste de usabilidade
sidebar_label: Testes
sidebar_position: 3
---

# Introdução

Nessa seção, serão abordados os testes de usabilidade feitos para validar a acessibilidade criada no projeto. Vale ressaltar que o projeto é feito majoritariamente para uso de idosos, então as diretrizes da WCAG levadas em consideração no desenvolvimento e nos testes focaram nesse público. Nesse teste foi validada apenas a tela de comunicação, pela qual o usuário consegue interagir com o LLM, focando principalmente no onboarding.

# Resultados dos testes:

[Tabela de Teste](https://docs.google.com/spreadsheets/d/1Xd88A07cx_FRDQgf0fCHbs82PogRik6lXO7LRSuTUDw/edit?gid=0#gid=0)

Pode ser observado que os testes não foram tão satisfatórios assim. Muitos dos testes não obtiveram resultados satisfatórios em conseguir utilizar a solução em si. Porém, também é notavel que os usuários acharam satisfatória a adaptabilidade feita para idosos. Sendo assim, identificamos alguns pontos principais levantados pelos testadores:

- `Falta de feedback`: Após enviar o audio, não há feedback satisfatório para entender que o áudio e a resposta estão sendo processados.

- `Falta de guia no onboarding`: Os usuários ficaram confusos perante a estrutura definida para o onboarding. Alguns pularam completamente a estrutura, enquanto outros não conseguiram sair do onboarding ou até não saber mais o que o J.A.R.B.A.S. está esperando ouvir.

- `Respostas muito longas`: O chatbot, às vezes, desvia da resposta esperada e envia respostas muito longas que não são interessantes ao usuário e acabam deformando a estrutura e estilização do site.

- `Erro ao enviar o audio`: Vários usuários encontraram erros relacionados ao backend da aplicação ao apertar sem querer o botão de envio de áudio.

Sendo assim, esses pontos geraram insights para como poder melhorar a experiência do usuário na nossa plataforma. Deve-se guiar o usuário melhor nas interações com o robô, colocar algo como um loading ao enviar audio, implementar um jeito de filtrar as mensagens para evitar o erro e diminuir a "criatividade" (termo usado para definir o quanto o chatbot pode pegar de informações aleatórias na conversa) do chatbot. 

# Conclusão

Com esses ajustes, deve-se melhorar bastante a experiência do usuário, visando o projeto a ficar mais intuitivo e com maior feedback para o usuário. Após os ajustes, deve ser feito outros testes para confirmar essa hipótese e verificar outras possíveis falhas na experiência do usuário.