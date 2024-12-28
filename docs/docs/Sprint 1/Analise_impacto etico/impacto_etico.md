---
title: Análise de impacto ético
sidebar_label: Análise de impacto ético
sidebar_position: 1
---

# Impactos da Tecnologia Desenvolvida na Sociedade e no Meio Ambiente: Aplicações em Robótica para Assistência Médica

&emsp;&emsp;Este capítulo analisa os potenciais impactos da tecnologia desenvolvida em parceria com a IBM, que visa criar robôs autônomos para auxiliar na assistência domiciliar de idosos. A proposta é que esses robôs ajudem na automação de tarefas diárias, sem substituir enfermeiros ou médicos, mas atuando como suporte no monitoramento de saúde e bem-estar.

&emsp;&emsp;O foco será analisar os efeitos nas dimensões de privacidade e proteção de dados, equidade e justiça, transparência e consentimento informado, responsabilidade social e viés e discriminação.

# 1. Privacidade e Proteção de Dados

&emsp;O robô desenvolvido para este projeto têm como uma de suas principais funcionalidades a coleta de dados para monitoramento de saúde e comportamento dos idosos (a princípio, vamos simular uma API de algum smartwatch para a obtenção dos dados de batimento cardíaco, pressão, dentre outros...) também, a coleta de dados através de LLM (Modelo de Linguagem Ampla) para entendermos as informações que consideramos relevantes como: "Como o idoso está se sentindo ?", hábitos, rotinas, check_up diário, dentre outras informações. Esses dados, embora importantes para o acompanhamento em tempo real, devem ser tratados com rigorosas medidas de privacidade, especialmente porque podem incluir informações sensíveis de saúde. Vale lembrar que os dados das pessoas/acompanhantes dos pacientes serão armazenados, a fim de serem usadas no contato emergencial.

## Justificativa para a categorização de dados sensíveis:

1. Os dados coletados envolvem aspectos de saúde, como lembretes de medicamentos, emergências e até interações que podem revelar o estado mental do paciente.

2. A Lei Geral de Proteção de Dados (LGPD) e o Regulamento Geral de Proteção de Dados (GDPR) exigem que esses dados sejam protegidos e processados com o consentimento explícito do usuário ou de seus responsáveis legais.

&emsp;&emsp;Caso o robô passe a coletar dados biométricos ou outras informações ainda mais sensíveis (uma vez desenvolvido e implementado a poc, essa seria uma possível atualziação do produto), serão adotadas medidas adicionais de segurança e conformidade, garantindo que todas as práticas estejam de acordo com as regulamentações vigentes.

# 2. Equidade e Justiça

&emsp;&emsp;O impacto dos robôs assistentes pode variar dependendo do perfil dos usuários e das condições em que são implementados. Como a tecnologia será utilizada em residências de idosos, é importante considerar as possíveis disparidades entre os grupos que terão acesso a essa solução.

## Impactos potenciais e formas de minimização de disparidades:

- **Idosos em situação de vulnerabilidade:** A tecnologia deve ser acessível a idosos de diferentes classes sociais. Projetos de inclusão digital e assistência financeira poderiam ser promovidos para garantir que o benefício seja estendido a todos. Se for o caso de obtenção do nosso produto através do convênio médico das pessoas, deve ser feito um entendimento da situação de todos para melhor acessibilidade.
- **Cuidadores e familiares:** O robô deve atuar como um facilitador para cuidadores e familiares, melhorando a qualidade de vida e aliviando o trabalho dos mesmos, sem criar um novo tipo de desigualdade digital entre aqueles que têm acesso à tecnologia e os que não têm. 

&emsp;&emsp;Vale lembrar que o robô desenvolvido não vai substituir o médico ou a enfermeira, mas será usado como um facilitador para lembrar o paciente de tomar os remédios no horário certo ou até mesmo um simples assistente para fazer companhia e conversar com os pacientes que se semtem sozinhos. A meta é que o projeto consiga seguir as diretrizes de design inclusivo para garantir que a tecnologia seja acessível e útil a todos, independentemente de sua origem socioeconômica.

# 3. Transparência e Consentimento Informado

&emsp;&emsp;Para garantir o uso ético dos robôs assistentes, será crucial manter a transparência sobre o que a tecnologia pode ou não pode fazer, e como os dados dos usuários serão tratados. Isso inclui comunicação clara com os idosos e seus familiares sobre as funcionalidades e limitações do robô.

## Princípios de transparência:

- **Objetivos e funcionalidades do robô:** O robô é um assistente, não um substituto de enfermeiros ou médicos. Seu papel principal é monitorar a saúde e fornecer suporte em emergências.
- **Limitações do robô:** Embora o robô seja capaz de notificar emergências e auxiliar no monitoramento de saúde, ele não fornece diagnósticos médicos nem realiza intervenções de emergência. Além disso, não acompanha o idoso fora de casa.

&emsp;Os usuários serão orientados a fornecer consentimento explícito para a coleta e uso dos dados de monitoramento, com a garantia de que terão total controle sobre essas informações.

# 4. Responsabilidade Social

&emsp;&emsp;A introdução de robôs na assistência domiciliar de idosos pode ter impactos positivos tanto para os indivíduos quanto para a sociedade como um todo, contribuindo para o bem-estar e o prolongamento da independência dos idosos.

## Potenciais efeitos positivos:

- **Redução da solidão:** Robôs que utilizam modelos de linguagem natural (LLMs) para interagir com os idosos podem fornecer suporte emocional, reduzindo os sentimentos de solidão e os efeitos psicológicos negativos associados à velhice.
- **Monitoramento contínuo:** O uso da tecnologia pode auxiliar no monitoramento constante da saúde dos idosos, fornecendo relatórios regulares para médicos e familiares, o que pode prevenir emergências e melhorar a qualidade do cuidado.

## Potenciais efeitos negativos:

- **Desemprego tecnológico:** Embora o robô não substitua médicos ou enfermeiros, a automação de algumas funções pode reduzir a demanda por cuidadores em determinadas circunstâncias e pode integrar ou  até mesmo substituir alguns equipamentos tecnológicos mesmo como uma Alexa, tendo em vista que o nosso robô pode navegar pela casa utilizando os seus spots específicos em cada cômodo e ela fica paradinha sempre, o que pode gerar impacto econômico. Uma forma de mitigar esse efeito, recomenda-se a criação de programas de requalificação e formação para os profissionais da área de saúde.

# 5. Viés e Discriminação

&emsp;&emsp;A coleta e uso de dados para treinamento de algoritmos que guiam as interações e decisões dos robôs deve ser feita com cautela para evitar viés algorítmico, especialmente em um contexto tão sensível quanto o da saúde dos idosos.

Análise de riscos de viés:

- **Viés na coleta de dados:** Se o robô interagir mais com certos grupos de idosos (por exemplo, aqueles mais tecnicamente instruídos), isso pode criar um viés nas interações e na qualidade dos insights gerados.

- **Mitigação de viés:** Para evitar isso, é fundamental diversificar as fontes de dados e realizar auditorias regulares para identificar qualquer tipo de discriminação ou tratamento desigual que possa surgir.

# Referências e fontes confiáveis

Organização Mundial da Saúde (OMS). Envelhecimento e Saúde. Disponível em: https://www.who.int/health-topics/ageing

Regulamento Geral de Proteção de Dados (GDPR). Normas e diretrizes para proteção de dados pessoais na União Europeia. Disponível em: https://gdpr-info.eu/

Lei Geral de Proteção de Dados (LGPD). Normas e regulamentações para proteção de dados pessoais no Brasil. Disponível em: https://www.planalto.gov.br/ccivil_03/_ato2015-2018/2018/lei/l13709.htm