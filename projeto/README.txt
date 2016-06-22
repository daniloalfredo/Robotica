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
´make real´. O executável resultante estará na pasta bin/

//***************************************************************

Log da Equipe:

- Dia 22/06/2016 (Anderson): 
	>>FEITO: 
		>>Versão inicial funcional do makefile para v-rep e robô.
		>>Alterada estrutura de pastas do projeto. Há no src pastas
		  para a API do VREP, a API do professor e a API própia nossa.
		>>Criei funcionalidade de ler mapa de arquivo. O arquivo de mapa
		  que está sendo utilizado é o ini/envmap.ini. Cada linha
		  representa um segmento de reta no mapa.
		>>Iniciada a integração do módulo detector de objetos da 
		  atividade2 ao projeto.
	>>A FAZER:
		>>Modificar o Robot.cpp para calcular corretamente a
		  estimativa xz de posição utilizando os sensores para o
		  robô andar com o método de kalman.
		>>Alterar o arquivo Simulation.ttt para ficar com as 
		  paredes de tamanho e posição equivalentes ao ambiente
		  real.
		>>Alterar o arquivo ini/envmap.ini para modelar o mapa real
		  no robô.
		>>Montar base de dados de imagens dos objetos que serão
		  utilizados tirando fotos com a câmera do robô.