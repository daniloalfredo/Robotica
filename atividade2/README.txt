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
na cena seu nome é mostrado na tela em tempo real e sua posicao
é mostrada com um círculo vermelho. Também é exibida a taxa de
quadros por segundo que está sendo alcançada.

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

Para que o software faça o treinamento basta executar o script 
"run.sh" com a flag "-train" ativa com o comando:
	´./run.sh -train´

O programa lê os objetos que estejam descritos no arquivo
"ini/database.ini". Este arquivo contém as seguintes 
informações (em ordem):

1) Número de objetos diferentes a serem reconhecidos

2) Para cada objeto:
	2.1) Número de imagens de treino desse objeto
	2.2) Nome do objeto
	2.3) Para cada imagem do objeto
		2.3.1)	Nome do arquivo de imagem com path
		
Exemplo de arquivo "ini/database.ini":

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
	
Adicionalmente pode-se alterar alguns parâmetros do detector no 
arquivo "ini/params.ini". Os parâmetros são, em ordem:

1) Confiança de detecção: valor de ponto flutuante entre 0 e 1
que representa percentagem mínima de confiança para o detector
considerar um objeto como detectado na cena.

2) Tamanho do dicionário: inteiro maior ou igual a 1 que representa
o número de features diferentes armazenadas no dicionário (utilizado
durante treinamento)
