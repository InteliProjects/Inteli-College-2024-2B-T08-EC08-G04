---
title: IBM Cloud
sidebar_label: IBM Cloud
sidebar_position: 1
---

# Documentação de Configuração no IBM Cloud: Criação do Watson Assistant e Actions

Este documento descreve os passos seguidos pelo grupo para configurar o IBM Watson Assistant no IBM Cloud e criar Actions para integrar na nossa aplicação.

---

## Criando o Serviço Watson Assistant

1. Após fazermos login no IBM Cloud, acessamos o **Catálogo** clicando em **"Catálogo"** no menu superior.
2. Na barra de pesquisa do catálogo, digitamos **"Assistant"** e selecionamos **"Watson Assistant"** nos resultados.
3. Seremos direcionados para a página de configuração do serviço Watson Assistant.
4. Selecionamos a região mais próxima de nós no campo **"Região"**.
5. Em **"Planos de precificação"**, selecionamos o plano **"Lite"** (gratuito) ou outro plano que atenda às nossas necessidades.
6. Clicamos em **"Criar"** para criar o serviço.

## Configurando o Watson Assistant

### Criando um Assistente

1. Após criar o serviço, seremos redirecionados para o painel do Watson Assistant.
2. Clicamos em **"Launch Watson Assistant"** para abrir a interface do assistente.
3. Na seção **"Assistentes"**, clicamos em **"Criar assistente"**.
4. Damos um nome ao nosso assistente, por exemplo, **"MeuAssistente"**.
5. Opcionalmente, adicionamos uma descrição.
6. Clicamos em **"Criar assistente"**.

### Adicionando Actions

1. Dentro do nosso assistente recém-criado, veremos a opção de adicionar habilidades (**Skills**).
2. Clicamos em **"Adicionar uma habilidade"**.
3. Selecionamos **"Actions"** como o tipo de habilidade.
4. Damos um nome à nossa Action, por exemplo, **"MinhaAction"**.
5. Clicamos em **"Criar"**.

#### Configurando Actions

1. Após criar a Action, seremos direcionados para o editor de Actions.
2. Clicamos em **"Adicionar ação"** para criar uma nova ação.
3. Damos um nome à ação, por exemplo, **"Saudação"**.
4. Definimos a condição de disparo da ação. Por exemplo, em **"Se o assistente reconhecer"**, digitamos **"Olá"**.
5. Na seção **"Então o assistente responde"**, inserimos a resposta que o assistente deve fornecer, por exemplo, **"Olá! Como posso ajudar?"**.
6. Clicamos em **"Salvar"** para salvar a ação.

Repetimos esses passos para adicionar mais ações conforme necessário.

## Obtendo as Credenciais do Serviço

Para integrar o Watson Assistant com nossa aplicação, precisaremos das credenciais da API.

1. Voltamos ao painel do IBM Cloud e navegamos até **"Lista de Recursos"** no menu lateral esquerdo.
2. Encontramos o serviço **"Watson Assistant"** que criamos.
3. Clicamos no serviço para acessar suas configurações.
4. No menu lateral, clicamos em **"Credenciais de serviço"**.
5. Se não houver credenciais criadas, clicamos em **"Nova credencial"**.
6. Damos um nome às credenciais ou deixamos o padrão.
7. Clicamos em **"Adicionar"**.
8. Expandimos a entrada de credenciais recém-criada clicando na seta ao lado do nome.
9. Veremos um JSON com as credenciais, incluindo **"apikey"** e **"url"**.
10. Anotamos os seguintes valores:
    - **API Key**: valor de **"apikey"**.
    - **Service URL**: valor de **"url"**.

### Obtendo o Assistant ID

1. No painel do Watson Assistant, navegamos até o assistente que criamos.
2. Clicamos nos três pontos (**...**) ao lado do nome do assistente.
3. Selecionamos **"Configurações"**.
4. Na aba **"Detalhes da API"**, encontraremos o **Assistant ID**.
5. Anotamos o **Assistant ID** para uso em nossa aplicação.

## Configurando o Ambiente de Desenvolvimento

Agora, devemos inserir as credenciais obtidas em nosso arquivo de ambiente (`.env`) na aplicação.

**Arquivo `.env`**

```env
WATSON_API_KEY=NOSSO_API_KEY
WATSON_SERVICE_URL=NOSSO_SERVICE_URL
WATSON_ASSISTANT_ID=NOSSO_ASSISTANT_ID

# Caso estejamos utilizando outros serviços como Text to Speech ou Speech to Text, incluímos suas credenciais:
TTS_API_KEY=NOSSO_TTS_API_KEY
TTS_SERVICE_URL=NOSSO_TTS_SERVICE_URL

STT_API_KEY=NOSSO_STT_API_KEY
STT_SERVICE_URL=NOSSO_STT_SERVICE_URL
```

Substituímos `NOSSO_API_KEY`, `NOSSO_SERVICE_URL` e `NOSSO_ASSISTANT_ID` pelos valores obtidos anteriormente.

## Conclusão

Configuramos com sucesso o IBM Watson Assistant no IBM Cloud e estamos prontos para integrá-lo à nossa aplicação. Com as Actions configuradas, nosso assistente pode responder a interações personalizadas. Certifiquemo-nos de proteger nossas credenciais e não compartilhá-las publicamente.

---

**Notas Adicionais:**

- Para utilizar serviços adicionais como Text to Speech ou Speech to Text, repetimos os passos de criação de serviço e obtemos as credenciais correspondentes.
- Consultamos a documentação oficial do [IBM Watson Assistant](https://cloud.ibm.com/docs/assistant?topic=assistant-getting-started) para aprofundar nosso conhecimento e explorar recursos avançados.
- Lembremo-nos de que o plano **"Lite"** tem limitações de uso. Se nossa aplicação exigir mais recursos, consideremos atualizar para um plano pago.

---
