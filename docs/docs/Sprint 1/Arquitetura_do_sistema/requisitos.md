---
title: Definição de requisitos
sidebar_label: Definição de requisitos
sidebar_position: 2
---

&emsp;&emsp;Nesta seção serão abordados os requisitos funcionais e não funcionais criados. Eles são fundamentais para definir as funcionalidades e guiar a execução do projeto. Os requisitos funcionais delineiam as ações fundamentais para o funcionamento do produto, enquanto os não funcionais complementam essas funcionalidades, incluindo aspectos como segurança e desempenho. Juntos, formam a base que orienta o desenvolvimento, garantindo uma compreensão clara do que será realizado e como as etapas do projeto serão conduzidas. Sendo assim, foram desenvolvidos duas tableas:

<p style={{textAlign: 'center'}}>Tabela 1 - Requisitos Funcionais (RF)</p>

| **Código do Requisito** | **Título**               | **Detalhes**                |
|-------------------------|--------------------------|-----------------------------|
| RF-01                   | Locomoção do robô         | Por meio do LIDAR e do SLAM, o robô deve ser capaz de se mover sozinho, seguindo o usuário pelos cômodos da casa evitando colisões. |
| RF-02                   | Conversação               | Por meio da aplicação WEB criada, o usuário deve ser capaz de conversar com o robô por meio de fala, recebendo a resposta também em fala. |
| RF-03                   | Responsável               | Por meio da aplicação WEB criada, o usuário deve ser capaz de definir um responsável pelo usuário, seja um médico, seja alguém de confiança. |
| RF-04                   | Emergência                | Por meio da aplicação WEB criada, o usuário deve ser capaz de emitir um alerta ao responsável definido para notificar que algo está errado na saúde do usuário. |
| RF-05                   | SLAM                      | Por meio do SLAM, o robô deve ser capaz de fazer reconhecimento dos cômodos da residência para identificar um local ideal por cômodo, onde o robô ficará. |
| RF-06                   | Notificação     | Por meio da aplicação WEB criada, o sistema deve ser capaz de avisar ao usuário sobre horários demarcados por ele. |
| RF-07                   | Check-up diário            | Por meio da aplicação WEB criada, o robô deve ser capaz de fazer perguntas ao usuário para check-up diário, mantendo o controle da saúde. |
| RF-08                   | Integração com sistemas de saúde | O sistema deve ser capaz de receber informações de APIs de sistemas externos como Apple Watch e similares, para fazer o controle da saúde física do usuário. |
| RF-09                   | Geração de relatório       | O sistema deve ser capaz de gerar relatórios sobre a saúde mental e física do usuário e enviar periodicamente esse relatório ao responsável definido. |

<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe J.A.R.B.A.S.</p>


<p style={{textAlign: 'center'}}>Tabela 2 - Requisitos Não Funcionais (RNF)</p>

| **Código do Requisito** | **Título**               | **Detalhes**                |
|----------------------|----------------------|-------------------------|
| RNF-01               | Tempo de resposta da colisão | O robô deve perceber colisões caso ele detecte algo 10 - 20 centimetros no caminho. |
| RNF-02               | Tempo de resposta da movimentação | O robô deve mudar seu trajeto ou parar o movimento em até 30ms após detectar algo no range de colisão. |
| RNF-03               | Tempo de resposta LLM | O tempo de resposta da resposta do chatbot pela aplicação WEB deve ser menor que 10 segundos, para manter a fluidez da conversa. |
| RNF-04               | Evitar falar dados sensíveis | O robô não deve falar dados sensíveis em relação ao usuário e a pessoa responsável definida. |
| RNF-05               | Acurácia do LLM | O robô deve entender no mínimo 70% do que o usuário está falando. |
| RNF-06               | Tempo de envio da emergência | O sistema deve enviar uma notificação de emergência em até 30 segundos depois do pedido de emergência manual. |
| RNF-07               | Asseguração da mensagem de emergência | O sistema deve assegurar que mesmo que haja algum problema no backend ou no envio do sinal de emergência, o próprio sistema deve identificar esse erro e assegurar que a mensagem chegará ao responsável. |
| RNF-08               | Horário da notificação | A notificação deve ser feita na hora exata que foi marcada. Sendo assim, caso o usuário marque um alarme as 16:30, o alarme deve tocar exatamente as 16:30. |
| RNF-09               | Tempo de coleta de dados externos | Os dados gerados por integrações com sistema de saúde devem ser coletados a cada 10 minutos. |
| RNF-10               | Recebimento de relatório | O relatório deve ser mandado ao responsável definido diariamente. |

<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe J.A.R.B.A.S.</p>

&emsp;&emsp;Estas tabelas apresentam os requisitos funcionais (RF) e não funcionais (RNF) relacionados ao uso do robô (Turtlebot) em função de robô de serviço, bem como a interface que o controla. 