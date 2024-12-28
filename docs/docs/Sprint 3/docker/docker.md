# Documentação de Dockerização da Aplicação Frontend

# Descrição do Processo

A seguir, documentamos o passo a passo para dockerizar uma aplicação frontend utilizando Node.js e Vite.

## 1. Estrutura do Projeto

A estrutura do projeto do frontend está organizada como segue:
```bash
frontend/
├── node_modules/
├── public/
├── src/
│   ├── pages/
│   │   ├── conversation.tsx
│   │   ├── home.tsx
│   ├── App.tsx
│   ├── ...
├── package.json
├── Dockerfile
├── docker-compose.yml
└── ...
```

* - **App.tsx:** Arquivo principal da aplicação.
* - **pages/:** Contém as páginas do frontend.
* - **Dockerfile:** Arquivo para criar a imagem Docker da aplicação.
* - **docker-compose.yml:** Arquivo de configuração para subir o container da aplicação.

---

## 2. Dockerfile

O arquivo Dockerfile é utilizado para criar a imagem Docker da aplicação.
```bash
# Use a imagem do Node.js
FROM node:18-alpine

# Define o diretório de trabalho no container
WORKDIR /app

# Copia os arquivos de dependência para o container
COPY package*.json ./

# Instala as dependências
RUN npm install

# Copia todos os arquivos do projeto para o container
COPY . .

# Expõe a porta 5173 para o frontend
EXPOSE 5173

# Comando para iniciar o servidor
CMD ["npm", "run", "dev", "--", "--host"]
```

* - **WORKDIR /app:** Define o diretório de trabalho dentro do container.
* - **COPY . .:** Copia todos os arquivos do projeto para o container.
* - **EXPOSE 5173:** Expõe a porta 5173 para que a aplicação esteja acessível.
* - **CMD:** Comando para rodar o servidor no container.

---

## 3. Docker Compose

O docker-compose.yml facilita o gerenciamento do container e o mapeamento de portas e volumes.
```bash
version: '3.8'
services:
  frontend:
    build:
      context: ./frontend
      dockerfile: Dockerfile
    ports:
      - "5173:5173"
    volumes:
      - ./frontend:/app
      - /app/node_modules
    command: ["npm", "run", "dev", "--", "--host"]
```

* - **build:** Define o caminho para o Dockerfile e a pasta do projeto.
* - **ports:** Mapeia a porta do container (5173) para o host.
* - **volumes:** Sincroniza o código-fonte do host com o container, excluindo o diretório node_modules.

---

## 4. Comandos Utilizados

Subir o container com Docker Compose
```bash
docker-compose up --build
```

# Vantagens da Dockerização

1. **Consistência do Ambiente:** O Docker garante que a aplicação funcione da mesma maneira em diferentes máquinas, eliminando problemas de incompatibilidade.

2. **Facilidade de Compartilhamento:** A imagem Docker pode ser compartilhada facilmente, permitindo que outros desenvolvedores rodem a aplicação sem configurações adicionais.

3. **Isolamento:** O container é independente do sistema operacional do host, garantindo um ambiente isolado e seguro.

4. **Escalabilidade:** Docker facilita o escalonamento horizontal da aplicação, permitindo a criação de múltiplos containers para lidar com a carga.

5. **Gerenciamento Simplificado:** Ferramentas como o *docker-compose* tornam o gerenciamento de múltiplos serviços mais eficiente.

# Desvantagens da Dockerização

1. **Curva de Aprendizado:** Desenvolvedores iniciantes podem achar o Docker complexo no começo.

2. **Overhead:** Apesar de ser leve, o Docker consome recursos adicionais do sistema, especialmente em sistemas com poucos recursos.

3. **Problemas de Volume:** Sincronizar volumes entre o host e o container pode gerar conflitos, como com o diretório node_modules.

4. **Dependência de Imagens:** A manutenção das imagens e versões pode se tornar desafiadora em projetos grandes.

# Conclusão

Dockerizar a aplicação frontend é uma prática recomendada para projetos que precisam de consistência e escalabilidade. Com o Docker, você pode isolar sua aplicação em um ambiente controlado e compartilhá-la facilmente com outros desenvolvedores ou servidores. No entanto, é essencial considerar o uso de recursos do sistema e a manutenção dos containers e imagens.

Neste caso, implementamos a dockerização utilizando um *Dockerfile* e um arquivo *docker-compose.yml*, garantindo que a aplicação frontend seja executada de maneira consistente e acessível em qualquer ambiente compatível com Docker.

Com esta configuração, é possível rodar o frontend facilmente em qualquer máquina, apenas utilizando os comandos documentados. Isso melhora a eficiência do desenvolvimento e a confiabilidade da entrega do produto.