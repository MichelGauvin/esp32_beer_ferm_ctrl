Voici la configuration pour nginx:

        server {
            listen 443 ssl;
            server_name nodemcu.mikesbrewshop.com;

            ssl_certificate /etc/nginx/certificate/fullchain1.pem;
            ssl_certificate_key /etc/nginx/certificate/privkey1.pem;

            location / {
                proxy_pass http://10.0.0.143;
                proxy_http_version 1.1;
                proxy_set_header Upgrade $http_upgrade;
                proxy_set_header Connection "upgrade";
                proxy_set_header Host $host;
            }

            location /ws {
                proxy_pass http://10.0.0.143:3333;  # WebSocket server port
                proxy_http_version 1.1;
                proxy_set_header Upgrade $http_upgrade;
                proxy_set_header Connection "upgrade";
                proxy_set_header Host $host;
            }
}
