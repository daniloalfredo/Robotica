#include "object_detector.h"

ObjectDetector::ObjectDetector()
{
	Init();
}

void ObjectDetector::Init()
{
	//Carrega parâmetros do detector
	LoadDetectorParams();

	//Carrega lista de objetos
	LoadObjects();

	//Verifica se existe um dicionário
	FileStorage file_dictionary(FILE_DICTIONARY, FileStorage::READ);
	
	//Se existe o dicionário carrega ele e carrega o svm
	if(file_dictionary.isOpened())
	{
		file_dictionary["vocabulary"] >> dictionary;
    	file_dictionary.release();
    	
    	svm.load(FILE_SVM);
	}
	
	//Se não há dicionário executa o treinamento
	else
		Train();
}

void ObjectDetector::LoadDetectorParams()
{
	FILE* file_params = fopen(FILE_PARAMS, "r");

	fscanf(file_params, "%lf", &confidence_threshold);
	fscanf(file_params, "%d", &dictionary_size);
	
	fclose(file_params);
}

void ObjectDetector::LoadObjects()
{
	//Abre arquivo com informações das imagens dos objetos
	FILE* file_database = fopen(FILE_DATABASE, "r");
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
		do{
		fscanf(file_database, " %[^\n]", aux);
		}while(strcmp(aux, "\n") == 0);
		string name(aux);
		
		//Pega o nome com path de cada imagem
		vector<string> image_filenames;
		for(int j = 0; j < num_images; j++)
		{
			do{
			fscanf(file_database, " %[^\n]", aux);
			}while(strcmp(aux, "\n") == 0);
			string filename(aux);
			image_filenames.push_back(filename);
			
			//Insere label no vetor de labels
			Mat label(1, 1, CV_32FC1, cv::Scalar(i));
			labels.push_back(label);
		}
		
		//Insere objeto na lista de objetos do detector
		Object new_object(name, image_filenames);
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
	TermCriteria tc(CV_TERMCRIT_ITER, 100, 0.001);
	int retries = 1;
	int flags = KMEANS_PP_CENTERS;
	BOWKMeansTrainer bowTrainer(dictionary_size, tc, retries, flags);
	
	dictionary = bowTrainer.cluster(featuresUnclustered);
	
	//Salva o dicionário no arquivo
	FileStorage file_dictionary(FILE_DICTIONARY, FileStorage::WRITE);
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
	svm.save(FILE_SVM);
	
	printf("End of Training\n");
}

Mat ObjectDetector::ComputeHistogram(Mat image)
{
	Ptr<DescriptorMatcher> matcher(new FlannBasedMatcher);
	Ptr<FeatureDetector> detector(new SiftFeatureDetector());
    Ptr<DescriptorExtractor> extractor(new SiftDescriptorExtractor); 
	BOWImgDescriptorExtractor bowDE(extractor, matcher);
	Mat image_histogram; 
	
	detector->detect(image, keypoints);
	bowDE.setVocabulary(dictionary);
    bowDE.compute(image, keypoints, image_histogram);
    
    return image_histogram;
}

void ObjectDetector::FindCenter(Mat frame, Point2f* center_pos, int object_class)
{
	Mat descriptors_frame;
	FlannBasedMatcher matcher;
  	vector<DMatch> matches;
  	SiftDescriptorExtractor detector;
  	
  	detector.compute(frame, keypoints, descriptors_frame);
  	matcher.match(descriptors_frame, objects[object_class].GetDescriptors(), matches);

  	double max_dist = 0; double min_dist = 50;

	for( int i = 0; i < descriptors_frame.rows; i++ )
	{ 
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

	std::vector< DMatch > good_matches;

	for( int i = 0; i < descriptors_frame.rows; i++ )
	{ 
		if(matches[i].distance <= max(2*min_dist, 0.02))
    		good_matches.push_back( matches[i]);
	}
	
	//---------------------------

	center_pos->x = 0;
	center_pos->y = 0;
	
	for(unsigned int i = 0; i < good_matches.size(); i++)
	{
		center_pos->x += keypoints[good_matches[i].queryIdx].pt.x;
		center_pos->y += keypoints[good_matches[i].queryIdx].pt.y;
	}	
	
	center_pos->x /= good_matches.size();
	center_pos->y /= good_matches.size();
}

string ObjectDetector::Detect(Mat frame, Point2f* center_pos)
{	
	//Computa histograma do frame segundo BoF
	Mat frame_histogram = ComputeHistogram(frame);
	
	//Classifica frame
	int prediction = svm.predict(frame_histogram);
	
	//Verifica a confiança da classificação
	double confidence = 1.0 / (1.0 + exp(-(svm.predict(frame_histogram, true))));
	printf("Detection Confidence: %lf%%\n", confidence*100);

	//Retorna o nome do objeto
	if(confidence <= confidence_threshold)
	{
		center_pos->x = -1;
		center_pos->y = -1;
		return string("Object Not Found");
	}
	
	else
	{
		FindCenter(frame, center_pos, prediction);
		return objects[prediction].GetName();
	}
}
