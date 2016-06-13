//***************************************************************

Implementação de Robô de Direção Diferencial simulado em software
V-REP 

//***************************************************************

Desenvolvido por:

Anderson Urbano (aafu@cin.ufpe.br)
Danilo Alfredo  (dams@cin.ufpe.br)
Diogo Rodrigues (drs3@cin.ufpe.br)
Heitor Rapela   (hrm@cin.ufpe.br)

//***************************************************************

Descrição:

Esta pasta contém o robô "Monstrinho" que utiliza o filtro de
kalman para se localizar no ambiente a partir de sua odometria
e da leitura de seus sensores juntamente com um mapa para
representar o ambiente.

//***************************************************************

Para executar:

Para executar o programa primeiro é preciso compilá-lo entrando
na pasta "Monstrinho" pelo terminal e executando o comando "make".

Após isso basta abrir o simulador V-REP e nele abrir o arquivo
de cena "Atividade3.ttt" na pasta principal e clicar no botão
de iniciar simulação. Nesse momento o robô ficará parado e 
será preciso executar no terminal o comando "./bin/monstrinho.bin"
para que o robô comece a se movimentar.


