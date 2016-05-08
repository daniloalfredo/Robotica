//***************************************************************

Software Detector de Objetos implementado em C++ com
OpenCV e OpenMP.

//***************************************************************

Desenvolvido por:

Anderson Urbano (aafu@cin.ufpe.br)
Danilo Alfredo  (dams@cin.ufpe.br)
Diogo Rodrigues (drs3@cin.ufpe.br)
Heitor Rapela   (hrm@cin.ufpe.br)

//***************************************************************

Descrição:

Este software utiliza o algoritmo Scale-invariant feature 
transform (SIFT) para extração de pontos de interesse das
imagens de treino e algoritmo Bag of Features (BoF) junto
com uma máquina de aprendizado SVM para aprender o conjunto
de objetos escolhidos.

O programa lê frames da câmera do computador e os processa
em busca dos objetos escolhidos. Se um objeto é reconhecido
na cena seu nome é mostrado na tela em tempo real.

//***************************************************************

Para executar:

Para executar o programa basta executar o script "run.sh" com
o comando: 
	´./run.sh´
O script irá compilar o software e executar em seguida. Em caso
de falta de permissão executar antes o commando:
	´chmod +x run.sh´
Nota: é necessário possuir instalado a biblioteca opencv
(o programa foi escrito utilizando a versão 2.4.12)
disponível em "http://opencv.org/" e compilador g++ com 
openmp.

//***************************************************************

Para treinar:

O programa lê os objetos que estejam descritos no arquivo "database.in"
Este arquivo contém as seguintes informações (em ordem):

1) Número de objetos diferentes a serem reconhecidos

2) Para cada objeto:
	2.1) Número de imagens de treino desse objeto
	2.2) Nome do objeto
	2.3) Para cada imagem do objeto
		2.3.1)	Nome do arquivo de imagem com path
		
Exemplo de arquivo "database.in":

	2

	2
	Sapato
	images/Sapato/1.jpg
	images/Sapato/2.jpg

	3
	Celular
	images/Celular/1.jpg
	images/Celular/2.jpg
	images/Celular/3.jpg
	
	
Este arquivo define que há dois objetos (um denominado Sapato
e outro denominado Celular). O primeiro objeto possui duas imagens
e o segundo 3 imagens que serão usadas no treinamento do detector.

Para que o software faça o treinamento é preciso além de alterar
o arquivo "database.in" remover o arquivo "dictionary.yml" da 
pasta do programa. Assim ao rodar o script de execução o programa
irá gerar um novo dicionário utilizando os objetos listados 
em "database.in".