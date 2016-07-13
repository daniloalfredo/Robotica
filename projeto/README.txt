//***************************************************************

Software de Robô para projeto final da disciplina de tópicos
avançados em inteligência robótica. 

//***************************************************************

Desenvolvido por:

Anderson Urbano (aafu@cin.ufpe.br)
Danilo Alfredo  (dams@cin.ufpe.br)
Diogo Rodrigues (drs3@cin.ufpe.br)
Heitor Rapela   (hrm@cin.ufpe.br)

//***************************************************************

Descrição:

Este projeto inclui software para o robô executar as tarefas de
se deslocar pelo cenário do teste e detectar objetos utilizando
sua câmera. Também é possível simular o robô utilizando o V-REP.

//***************************************************************

Para executar:

Para compilar o robô para V-REP basta utilizar o comando ´make sim´
e para compilar o robô para o dispositivo real basta utilizar 
´make real´. Os arquivos fontes da pasta src serão compilados e 
terão um arquivo executável equivalente na pasta bin.

Para executar um programa pasta estando na pasta raiz do projeto
utilizar o comando: ./bin/<nome do programa>.bin

Exemplo: ./bin/remoteControl.bin

No caso do robô real é necessário utilizar sudo antes do comando
para rodar com permissões de administrador.

//***************************************************************

Inicialização:

Há vários arquivos de inicialização na pasta ini o arquivo
hardware.ini controla alguns parâmetros do robô como o seu
PID.