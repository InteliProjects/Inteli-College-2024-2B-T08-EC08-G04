---
sidebar_label: Adequação à LGPD
sidebar_position: 5
---

# Adequação à LGPD

&emsp;&emsp;A LGPD (Lei Geral de Proteção de Dados) é uma lei que entrou em vigor em 2018 e que legisla sobre a proteção de informações de pessoas naturais no Brasil. A lei define que qualquer tratamento (isto é, qualquer tipo de operação) feito sobre dados pessoais e dados sensíveis deve atender a uma série de critérios em prol da preservação dos direitos de todos atingidos pela legislação brasileira.
	
&emsp;&emsp;À vista da importância do atendimento a esses critérios para adequar a solução desenvolvida à legislação brasileira e otimizar a experiência do usuário de modo a fornecer a sensação de segurança durante sua utilização, a equipe J.A.R.B.A.S. definiu e registrou na presente seção como o sistema se conformará a LGPD.

&emsp;&emsp;Primeiramente, deve-se considerar os quatro principais envolvidos — direta ou indiretamente — no uso da solução final e quais dados são coletados de cada um desses, conforme demonstra a tabela abaixo:

<p style={{textAlign: 'center'}}> Tabela 1 - Envolvidos no uso da solução J.A.R.B.A.S. e seus dados coletados</p>

| **Indivíduo/Instituição**    | **Dados fornecidos**                                                                                                                                                        | **Dados coletados**                                                                                                                                                                         |
|------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Operadoras de plano de saúde | Dados de identificação e dados médicos (comorbidades, medicações etc) de beneficiários.                                                                                     | Dados de identificação, dados médicos (comorbidades, medicações etc) e dados financeiros de beneficiários. Dados institucionais e financeiras para o caso de planos de saúde para empresas. |
| Médico                       | Dados de identificação e CRM (número de identificação do Conselho Regional de Medicina).                                                                                    | Dados de identificação, comprovante de cadastro no plano de saúde, informações sobre emoções, sensações físicas, hábitos alimentares, comorbidades e medicações prescritas.                 |
| Pessoa idosa                 | Dados de identificação, comprovante de cadastro no plano de saúde, informações sobre emoções, sensações físicas, hábitos alimentares, comorbidades e medicações prescritas. | N/A                                                                                                                                                                                         |
| Responsável                  | Dados de identificação e dados de contato.                                                                                                                                  | N/A                                                                                                                                                                                         |

<p style={{textAlign: 'center'}}>Fonte: Elaborado pela equipe J.A.R.B.A.S.</p>

&emsp;&emsp;O mapeamento proposto na tabela é fundamental para que, ao longo do desenvolvimento da solução, a equipe tenha clareza sobre quais dados serão necessários para cada tipo de usuário e como garantir a conformidade com os seguintes princípios da LGPD:

- **Finalidade e adequação:** O tratamento dos dados será planejado exclusivamente para cumprir a proposta do sistema, que é reduzir a solidão de pessoas idosas e facilitar o acompanhamento médico destas. Portanto, os dados serão utilizados de forma limitada ao que for essencial para a prestação destes serviços. A revisão deste princípio deve ser feita regularmente durante o desenvolvimento e após o lançamento da solução, a fim de assegurar que não haja coleta e uso desnecessários de informações.

- **Transparência e consentimento:** A equipe J.A.R.B.A.S. deve se assegurar que todos os envolvidos que constam na tabela 1 sejam informados de maneira clara e transparente sobre a finalidade da coleta de seus dados, sendo necessário o consentimento explícito para o tratamento de dados pessoais e sensíveis, especialmente para os beneficiários idosos. A partir da própria interface do aplicativo mobile, que conta com uma tela específica para informar o usuário sobre essa coleta, esses consentimentos serão registrados e arquivados, respeitando os requisitos da LGPD e facilitando futuras auditorias.

- **Segurança e confidencialidade:** A arquitetura da solução será projetada com diversas camadas de segurança para proteger as informações de acessos não autorizados, vazamentos e qualquer outro tipo de incidente. Serão implementadas medidas como criptografia, controle de acesso e monitoramento contínuo, e a equipe se comprometerá a realizar auditorias regulares e adotar práticas de segurança proativas para minimizar os riscos.

- **Direitos dos titulares:** Desde o início, será assegurado que todos os titulares de dados envolvidos na solução tenham o direito de solicitar o acesso, correção, anonimização ou exclusão de seus dados, conforme previsto na LGPD. Um canal específico será planejado para atender a essas solicitações, com prazos e procedimentos definidos para cada tipo de atendimento, de modo a garantir a conformidade da solução.

&emsp;&emsp;Diante do exposto, define-se que a adequação da solução à LGPD será um compromisso contínuo que guiará o desenvolvimento da solução. Cabe à equipe J.A.R.B.A.S., então, alinhar o desenvolvimento do projeto com práticas de conformidade e revisões constantes, assegurando que as operações de tratamento de dados atendam aos requisitos legais e respeitem a privacidade e segurança dos futuros usuários. Dessa forma, a equipe poderá criar um produto confiável, seguro e em conformidade com a legislação de proteção de dados vigente no Brasil.