require('dotenv').config();
const jwt = require('jsonwebtoken');

// Chave secreta do Supabase
const jwtSecret = process.env.JWT_SECRET;

function authMiddleware(req, res, next) {
    try {
        // Captura o token do cabeçalho Authorization
        const authHeader = req.headers['authorization'];
        if (!authHeader) {
            return res.status(403).json({ message: 'Token necessário para acessar esta rota' });
        }

        // Remove o prefixo "Bearer", se necessário
        const token = authHeader.startsWith('Bearer ')
            ? authHeader.split(' ')[1]
            : authHeader;

        // Verifica e decodifica o token JWT
        jwt.verify(token, jwtSecret, (err, decoded) => {
            if (err) {
                return res.status(403).json({ message: 'Token inválido ou expirado' });
            }

            // Decodificação bem-sucedida, armazena os dados do usuário
            req.user = decoded;
            next(); // Passa para o próximo middleware ou rota
        });
    } catch (error) {
        console.error('Erro no authMiddleware:', error);
        return res.status(500).json({ message: 'Erro interno no servidor' });
    }
}

module.exports = authMiddleware;
