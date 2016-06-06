Este projeto apresenta um exemplo de como utilizar a API remota (rede) do V-REP para controlar o robô de direção diferencial.

Passos:

1 - Configure o IP da máquina onde o está V-REP (127.0.0.1, caso esteja na mesma máquina):

	#define V_REP_IP_ADDRESS "10.0.2.2"//"127.0.0.1"

2 - Configure a porta utilizada pelo V-REP, geralmente 19997 ou 1999:

	#define V_REP_PORT 19997//1999;

3 - Execute o programa, ele irá:
  - Tentar se conectar com o V-REP;
  - Iniciar a simulação;
  - Encerrar o programa se a simulação for parada no V-REP (botão stop).

