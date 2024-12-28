# Usar imagem base do Rasa SDK que já inclui o suporte para ações
FROM rasa/rasa-sdk:3.1.0

USER root

# Atualizar pip e setuptools
RUN pip install --upgrade pip setuptools

# Instalar wheel para evitar problemas de instalação com fire e outros pacotes
RUN pip install wheel

# Instalar versão específica do fire que evita problemas de instalação
RUN pip install fire==0.4.0

# Instalar Rasa
RUN pip install rasa

# Instalar Rasa e dependências de ações, se houver
RUN pip install rasa

USER 1001

# Definir diretório de trabalho
WORKDIR /app

# Copiar arquivos do projeto para o contêiner
COPY . /app

# Expor as portas necessárias 
EXPOSE 5005 5055

# Comando para iniciar o Rasa e o servidor de ações
# COPY entrypoint.sh /app/entrypoint.sh
# RUN chmod +x /app/entrypoint.sh
CMD ["/app/entrypoint.sh"]

