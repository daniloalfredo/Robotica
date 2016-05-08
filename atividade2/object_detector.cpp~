#include "object_detector.h"

void ObjectDetector::Init(double confidence)
{
	confidence_threshold = confidence;

	//Carrega lista de objetos
	LoadObjects();

	//Verifica se existe um dicionário
	FileStorage file_dictionary("dictionary.yml", FileStorage::READ);
	
	//Se existe o dicionário carrega ele e carrega o svm
	if(file_dictionary.isOpened())
	{
		file_dictionary["vocabulary"] >> dictionary;
    	file_dictionary.release();
    	
    	svm.load("svm_data.in");
	}
	
	//Se não há dicionário executa o treinamento
	else
		Train();
}

void ObjectDetector::LoadObjects()
{
	//Abre arquivo com informações das imagens dos objetos
	FILE* file_database = fopen("database.in", "r");
	int num_objects;
	
	//Verifica quantos objetos diferentes existem
	fscanf(file_database, "%d", &num_objects);
	char aux[100];
	
	for(int i = 0; i < num_objects; i++)
	{
		//Pega o número de imagens do objeto
		int num_images;
		fscanf(file_database, "%d", &num_images);
		
		//Pega o nome do objeto
		fscanf(file_database, " %[^\n]", aux);
		string name(aux);
		
		//Pega o nome com path de cada imagem
		vector<string> image_filenames;
		for(int j = 0; j < num_images; j++)
		{
			fscanf(file_database, " %[^\n]", aux);
			string filename(aux);
			image_filenames.push_back(filename);
			
			//Insere label no vetor de labels
			Mat label(1, 1, CV_32FC1, cv::Scalar(i));
			labels.push_back(label);
		}
		
		//Insere objeto na lista de objetos do detector
		Object new_object(i, name, image_filenames);
		objects.push_back(new_object);
	}
	
	fclose(file_database);
}

void ObjectDetector::Train()
{   
	printf("Creating Dictionary...\n");
	
	SiftDescriptorExtractor detector;
	Mat featuresUnclustered;
	
	//Lê cada imagem do banco para extrair descritores
	for(unsigned int i = 0; i < objects.size(); i++)
	{
		vector<string> image_filenames = objects[i].GetFilenames();
		
		for(unsigned int j = 0; j < image_filenames.size(); j++)
		{
			//Abre a imagem em escala de cinza
			Mat image = imread(image_filenames[j], CV_LOAD_IMAGE_GRAYSCALE);
			
			//Detecta os pontos de interesse
			vector<KeyPoint> keypoints;
			detector.detect(image, keypoints);
		
			//Computa os descritores para cada ponto de interesse
			Mat descriptor;
			detector.compute(image, keypoints, descriptor);
			
			//Guarda os descritores encontrados
			featuresUnclustered.push_back(descriptor);     
		}
	}
	
	//Constroi o dicionário para o BoF
	int dictionarySize = 30;
	TermCriteria tc(CV_TERMCRIT_ITER, 100, 0.001);
	int retries = 1;
	int flags = KMEANS_PP_CENTERS;
	BOWKMeansTrainer bowTrainer(dictionarySize, tc, retries, flags);
	
	dictionary = bowTrainer.cluster(featuresUnclustered);
	
	//Salva o dicionário no arquivo
	FileStorage file_dictionary("dictionary.yml", FileStorage::WRITE);
	file_dictionary << "vocabulary" << dictionary;
	file_dictionary.release();
	
	//Prepara matriz de treinamento do SVM
	printf("Training SVM...\n");
	Mat training_matrix;
	
	for(unsigned int i = 0; i < objects.size(); i++)
	{
		vector<string> image_filenames = objects[i].GetFilenames();
		
		for(unsigned int j = 0; j < image_filenames.size(); j++)
		{
			//Abre a imagem em escala de cinza
			Mat image = imread(image_filenames[j], CV_LOAD_IMAGE_GRAYSCALE);
			
			//Computa o histograma segundo o BoF
			Mat image_histogram = ComputeHistogram(image);
			
			//Adiciona à matrix de treinamento
			training_matrix.push_back(image_histogram);  
		}
	}
	
	//Treina SVM e salva em arquivo
	CvSVMParams params;
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = CvSVM::RBF;
	params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	
	svm.train(training_matrix, labels, Mat(), Mat(), params);
	svm.save("svm_data.in");
	
	printf("End of Training\n");
}

Mat ObjectDetector::ComputeHistogram(Mat image)
{
	Ptr<DescriptorMatcher> matcher(new FlannBasedMatcher);
	Ptr<FeatureDetector> detector(new SiftFeatureDetector());
    Ptr<DescriptorExtractor> extractor(new SiftDescriptorExtractor); 
	BOWImgDescriptorExtractor bowDE(extractor, matcher);
	vector<KeyPoint> keypoints;
	Mat image_histogram; 
	
	detector->detect(image, keypoints);
	bowDE.setVocabulary(dictionary);
    bowDE.compute(image, keypoints, image_histogram);
    
    return image_histogram;
}

string ObjectDetector::Detect(Mat frame)
{	
	//Computa histograma do frame segundo BoF
	Mat frame_histogram = ComputeHistogram(frame);
	
	//Classifica frame
	int prediction = svm.predict(frame_histogram);
	
	//Verifica a confiança da classificação
	double confidence;
	
	try {
		confidence = 1.0 / (1.0 + exp(-(svm.predict(frame_histogram, true))));
		printf("Detection Confidence: %lf%%\n", confidence*100);
	}catch(cv::Exception){
		printf("Exception Ocurred\n");
		confidence = 0.0;
	}

	//Retorna o nome do objeto
	if(confidence <= confidence_threshold)
		return string("Object Not Found");
	else
		return objects[prediction].GetName();
}
