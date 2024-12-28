---
title: Requisitos Funcionais e Não Funcionais Atendidos
sidebar_label: Requisitos Atendidos
sidebar_position: 1
---

# Introdução

&emsp;&emsp;Nessa seção serão abordados os requisitos funcionais e não funcionais pré-definidos [aqui](https://inteli-college.github.io/2024-2B-T08-EC08-G04/docs/Sprint%201/Arquitetura_do_sistema/requisitos) que foram atingidos pelo projeto.

# Requisitos Funcionais

Essa seção conterá listas sobre os requisitos funcionais atendidos completamente, parcialmente atendidos e não atendidos.

## Requisitos Funcionais Atendidos

- `RF-01`: Locomoção do robô 

- `RF-02`: Comunicação com o chatbot

- `RF-03`: Definição de responsável

- `RF-07`: Check-up diário

## Requisitos Funcionais Parcialmente Atendidos

- `RF-04`: Alerta de emergência
    - O chatbot reconhece o alerta, porém é mandado para um json a mensagem, ao invés de mandar ao responsável

- `RF-05`: SLAM
    - O robô consegue mapear o mapa, porém ele não define um ponto ideal para ficar no mapa criado

- `RF-09`: Criação de relatórios
    - A formação dos relatórios está formada, porém os formulários não são gerados automaticamente

## Requisitos Funcionais Não Atendidos

- `RF-06`: Alarme

- `RF-08`: Integração com sistemas de saúde externo

# Requisitos Não Funcionais

Essa seção conterá listas sobre os requisitos não funcionais atendidos completamente, parcialmente atendidos e não atendidos.

## Requisitos Funcionais Atendidos

- `RNF-01`: Tempo de resposta da colisão

- `RNF-02`: Tempo de resposta da movimentação

- `RNF-04`: Evitar falar dados sensíveis

- `RNF-05`: Acurácia do LLM

## Requisitos Não Funcionais Parcialmente Atendidos

- `RNF-03`: Tempo de resposta LLM
    - Algumas vezes o chatbot demora mais de 10s, porém na maioria dos testes foi menor que 10s

- `RNF-06`: Tempo de envio da emergência
    - O sistema de emergência demora menos que 30s, porém como não é enviado a mensagem de emrgência ao usuário, não tem como medir o tempo de envio da mensagem em si

## Requisitos Funcionais Não Atendidos

- `RNF-07`: Asseguração da mensagem de emergência

- `RNF-08`: Horário da notificação

- `RNF-09`: Tempo de coleta de dados externos

- `RNF-10`: Recebimento de relatório

# Conclusão 

&emsp;&emsp;Como pode ser observado acima, a maioria dos requisitos funcionais foi atendida com sucesso, garantindo que o projeto cumpriu suas funções essenciais. Entretanto, a taxa de sucesso foi menor nos requisitos não funcionais, indicando que ainda há espaço para otimizações importantes.

&emsp;&emsp;Os requisitos funcionais parcialmente atendidos sugerem que o projeto possui boas bases implementadas, mas carece de refinamento em funcionalidades mais avançadas, como o envio efetivo de alertas e a geração automática de formulários. Já os não funcionais parcialmente e não atendidos apontam áreas que precisam de atenção para garantir a estabilidade e eficiência do sistema.