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
		printf("Loading Dictionary...\n");
		file_dictionary["vocabulary"] >> dictionary;
    	file_dictionary.release();
    	
    	printf("Loading SVM...\n");
    	svm.load(FILE_SVM);
	}
	
	//Se não há dicionário executa o treinamento
	else
	{
		if(!use_advanced_training)
			Train();
		else
			TrainAdvanced();
	}
}

void ObjectDetector::LoadDetectorParams()
{
	char aux[50];

	printf("Loading Params...\n");
	FILE* file_params = fopen(FILE_PARAMS, "r");

	fscanf(file_params, "%*[^:] %*c %lf", &confidence_threshold);
	
	fscanf(file_params, "%*[^:] %*c %[^\n]", aux);
	if(strcmp(aux, "SIFT") == 0)
		descriptor_extractor = 0;
	else
		descriptor_extractor = 1;
	
	fscanf(file_params, "%*[^:] %*c %d", &dictionary_size);
	fscanf(file_params, "%*[^:] %*c %d", &blur_size);
	
	int aux_adv_training;
	fscanf(file_params, "%*[^:] %*c %d", &aux_adv_training);
	use_advanced_training = (aux_adv_training != 0);
	
	fscanf(file_params, "%*[^:] %*c %d", &num_svms_for_advanced_training);
	
	fscanf(file_params, "%*[^:] %*c %[^\n]", aux);
	if(strcmp(aux, "LINEAR") == 0)
		svm_kernel_type = CvSVM::LINEAR;
	
	else if(strcmp(aux, "POLY") == 0)
		svm_kernel_type = CvSVM::POLY;	
	else if(strcmp(aux, "SIGMOID") == 0)
		svm_kernel_type = CvSVM::SIGMOID;
	else
		svm_kernel_type = CvSVM::RBF;
		
	fscanf(file_params, "%*[^:] %*c %d", &svm_degree);
	fscanf(file_params, "%*[^:] %*c %d", &svm_gamma);
	
	fclose(file_params);
	
	printf("\tCONFIDENCE THRESHOLD: %.2lf\n", confidence_threshold);
	if(descriptor_extractor == 0)
		printf("\tDESCRIPTOR EXTRACTOR: SIFT\n");
	else
		printf("\tDESCRIPTOR EXTRACTOR: SURF\n");
	printf("\tDICTIONARY SIZE: %d\n", dictionary_size);
	printf("\tBLUR WINDOW SIZE: %d\n", blur_size);
	printf("\tUSE ADVANCED TRAINING: %d\n", (int) use_advanced_training);
	printf("\tNUM OF SVMs FOR ADVANCED TRAINING: %d\n", num_svms_for_advanced_training);
	printf("\tSVM KERNEL TYPE: %s\n", aux);
	printf("\tSVM DEGREE: %d\n", svm_degree);
	printf("\tSVM GAMMA: %d\n", svm_gamma);
}

void ObjectDetector::LoadObjects()
{
	printf("Loading Objects...\n");

	//Abre arquivo com informações das imagens dos objetos
	FILE* file_database = fopen(FILE_DATABASE, "r");
	int num_objects;
	
	//Verifica quantos objetos diferentes existem
	fscanf(file_database, "%*[^:] %*c %d", &num_objects);
	char aux[100];
	
	for(int i = 0; i < num_objects; i++)
	{
		//Pega o nome do objeto
		fscanf(file_database, "%*[^:] %*c %[^\n]", aux);
		string name(aux);
		
		printf("\tObject: %s\n", name.c_str());
	
		//Pega o número de imagens do objeto
		int num_images;
		fscanf(file_database, "%*[^:] %*c %d", &num_images);
		
		//Pega o nome com path de cada imagem
		vector<string> image_filenames;
		for(int j = 0; j < num_images; j++)
		{
			fscanf(file_database, "%*[^:] %*c %[^\n]", aux);
			string filename(aux);
			image_filenames.push_back(filename);
			
			//Insere label no vetor de labels
			Mat label(1, 1, CV_32FC1, cv::Scalar(i));
			labels.push_back(label);
		}
		
		//Insere objeto na lista de objetos do detector
		Object new_object(name, image_filenames, descriptor_extractor);
		objects.push_back(new_object);
	}
	
	fclose(file_database);
}

void ObjectDetector::Train()
{   
	printf("Creating Dictionary...\n");
	
	SiftDescriptorExtractor detector_sift;
	SurfDescriptorExtractor detector_surf;
	Mat featuresUnclustered;
	
	//Lê cada imagem do banco para extrair descritores
	for(unsigned int i = 0; i < objects.size(); i++)
	{
		vector<string> image_filenames = objects[i].GetFilenames();
		
		for(unsigned int j = 0; j < image_filenames.size(); j++)
		{
			//Abre a imagem em escala de cinza
			Mat image = imread(image_filenames[j], CV_LOAD_IMAGE_GRAYSCALE);
			
			//Passa filtro gaussiano na imagem
			if(blur_size > 0)
				GaussianBlur(image, image, Size(blur_size, blur_size), 0, 0);
			
			//Detecta os pontos de interesse
			vector<KeyPoint> keypoints;
			
			if(descriptor_extractor == 0)
				detector_sift.detect(image, keypoints);
			else
				detector_surf.detect(image, keypoints);
		
			//Computa os descritores para cada ponto de interesse
			Mat descriptor;
			
			if(descriptor_extractor == 0)
				detector_sift.compute(image, keypoints, descriptor);
			else
				detector_surf.compute(image, keypoints, descriptor);
			
			//Guarda os descritores encontrados
			featuresUnclustered.push_back(descriptor); 
			
			//Salva log do treino
			SaveKeypointImageLog(image, keypoints, i, j);    
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
			
			//Passa filtro gaussiano na imagem
			if(blur_size > 0)
				GaussianBlur(image, image, Size(blur_size, blur_size), 0, 0);
		
			//Computa o histograma segundo o BoF
			Mat image_histogram = ComputeHistogram(image);
			
			//Adiciona à matrix de treinamento
			training_matrix.push_back(image_histogram);  
		}
	}
	
	//Treina SVM e salva em arquivo
	CvSVMParams params;
	params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = svm_kernel_type; //LINEAR / POLY / RBF / SIGMOID
	params.degree = svm_degree;	//influencia kernel POLY
	params.gamma = svm_gamma;	//influencia kernels  POLY / RBF / SIGMOID
	
	svm.train(training_matrix, labels, Mat(), Mat(), params);
	svm.save(FILE_SVM);
}

void ObjectDetector::TrainAdvanced()
{
	printf("Starting Advanced Training...\n");
	SiftDescriptorExtractor detector_sift;
	SurfDescriptorExtractor detector_surf;
	int best_svm_index = 0;
	int best_svm_hits = 0;
	
	//Parametros de dicionario
	TermCriteria tc(CV_TERMCRIT_ITER, 100, 0.001);
	int retries = 1;
	int flags = KMEANS_PP_CENTERS;
	BOWKMeansTrainer bowTrainer(dictionary_size, tc, retries, flags);
	
	//Parametros da SVM
	CvSVMParams params;
	params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = svm_kernel_type; //LINEAR / POLY / RBF / SIGMOID
	params.degree = svm_degree;	//influencia kernel POLY
	params.gamma = svm_gamma;	//influencia kernels  POLY / RBF / SIGMOID
	
	for(int it = 1; it <= num_svms_for_advanced_training; it++)
	{
		Mat adv_labels;
		Mat featuresUnclustered;
		vector<vector<string> > image_filenames;
	
		printf("\tCreating Dictionary for SVM %d...\n", it);
	
		//Lê um subconjunto de imagens de cada objeto e extrai descritores
		for(unsigned int i = 0; i < objects.size(); i++)
		{
			image_filenames.push_back(GetSubsetOfImages(i));
		
			for(unsigned int j = 0; j < image_filenames[i].size(); j++)
			{
				//Insere label no vetor de labels
				Mat label(1, 1, CV_32FC1, cv::Scalar(i));
				adv_labels.push_back(label);
			
				//Abre a imagem em escala de cinza
				Mat image = imread(image_filenames[i][j], CV_LOAD_IMAGE_GRAYSCALE);
			
				//Passa filtro gaussiano na imagem
				if(blur_size > 0)
					GaussianBlur(image, image, Size(blur_size, blur_size), 0, 0);
			
				//Detecta os pontos de interesse
				vector<KeyPoint> keypoints;
				if(descriptor_extractor == 0)
					detector_sift.detect(image, keypoints);
				else
					detector_surf.detect(image, keypoints);
		
				//Computa os descritores para cada ponto de interesse
				Mat descriptor;
				if(descriptor_extractor == 0)
					detector_sift.compute(image, keypoints, descriptor);
				else
					detector_surf.compute(image, keypoints, descriptor);
			
				//Guarda os descritores encontrados
				featuresUnclustered.push_back(descriptor);   
			}
		}
		
		//Constroi o dicionário para o BoF
		dictionary = bowTrainer.cluster(featuresUnclustered);
	
		//Treina uma SVM
		Mat training_matrix;
		for(unsigned int i = 0; i < objects.size(); i++)
		{		
			for(unsigned int j = 0; j < image_filenames[i].size(); j++)
			{
				//Abre a imagem em escala de cinza
				Mat image = imread(image_filenames[i][j], CV_LOAD_IMAGE_GRAYSCALE);
			
				//Passa filtro gaussiano na imagem
				if(blur_size > 0)
					GaussianBlur(image, image, Size(blur_size, blur_size), 0, 0);
		
				//Computa o histograma segundo o BoF
				Mat image_histogram = ComputeHistogram(image);
			
				//Adiciona à matrix de treinamento
				training_matrix.push_back(image_histogram);  
			}
		}
		
		printf("\tTraining SVM %d...\n", it);
		svm.train(training_matrix, adv_labels, Mat(), Mat(), params);
		
		//Salva svm num arquivo temporário
		char temp_svm_filename[50];
		sprintf(temp_svm_filename, "log/temp_svm/%d.ini", it);
		svm.save(temp_svm_filename);
		
		//Testa a nova svm
		printf("\tValidating SVM %d...\n", it);
		int hits = ValidateSVM();
		printf("\tValidation Results for SVM %d: %d hits\n", it, hits);
		
		if(hits > best_svm_hits)
		{
			best_svm_hits = hits;
			best_svm_index = it;
		}
	}
	
	//Salva o dicionário no arquivo
	FileStorage file_dictionary(FILE_DICTIONARY, FileStorage::WRITE);
	file_dictionary << "vocabulary" << dictionary;
	file_dictionary.release();
	
	//Recupera melhor svm encontrada
	char temp_svm_filename[50];
	sprintf(temp_svm_filename, "log/temp_svm/%d.ini", best_svm_index);
	svm.load(temp_svm_filename);
	
	//Salva a melhor svm encontrada corretamente
	svm.save(FILE_SVM);
	
	//Printa resultado do treinamento
	printf("End of Training. Chosen SVM: %d. Hits: %d\n", best_svm_index, best_svm_hits);
}

vector<string> ObjectDetector::GetSubsetOfImages(int obj_id)
{
	vector<string> subset;
	int total_images = objects[obj_id].GetNumImages()-1;
	
	//Escolhe um número de imagens pra usar
	int num_rand_images = 5;
	
	if(total_images >= total_images)
		num_rand_images = total_images;
	
	vector<string> randomized_filenames = objects[obj_id].GetFilenames();
	random_shuffle(randomized_filenames.begin(), randomized_filenames.end());
	
	for(int i = 0; i < num_rand_images; i++)
		subset.push_back(randomized_filenames[i]);
			
	return subset;
}

int ObjectDetector::ValidateSVM()
{
	int hits = 0;

	for(unsigned int i = 0; i < objects.size(); i++)
	{
		//Abre a imagem de validação
		Mat image = objects[i].GetValidationImage().clone();
		
		//Passa filtro gaussiano
		if(blur_size > 0)
			GaussianBlur(image, image, Size(blur_size, blur_size), 0, 0);

		//Computa histograma do frame segundo BoF
		Mat image_histogram = ComputeHistogram(image);

		//Classifica imagem
		int prediction = svm.predict(image_histogram);
	
		if(objects[prediction].GetName() == objects[i].GetName())
			hits++;
	}
	
	return hits;
}

void ObjectDetector::SaveKeypointImageLog(Mat image, vector<KeyPoint>keypoints, unsigned int i, unsigned int j)
{
	drawKeypoints(image, keypoints, image, Scalar(2,254,255), DrawMatchesFlags::DEFAULT);
	char path[100];
	sprintf(path, "log/obj%d_img%d.jpg", i, j); 
	imwrite(path, image);
}

Mat ObjectDetector::ComputeHistogram(Mat image)
{
	Ptr<DescriptorMatcher> matcher(new FlannBasedMatcher);
	Ptr<FeatureDetector> detector_sift(new SiftFeatureDetector());
	Ptr<FeatureDetector> detector_surf(new SurfFeatureDetector());
	Ptr<DescriptorExtractor> extractor_sift(new SiftDescriptorExtractor); 
    Ptr<DescriptorExtractor> extractor_surf(new SurfDescriptorExtractor); 
	BOWImgDescriptorExtractor bowDE_sift(extractor_sift, matcher);
	BOWImgDescriptorExtractor bowDE_surf(extractor_surf, matcher);
	Mat image_histogram; 
	
	if(descriptor_extractor == 0)
	{
		detector_sift->detect(image, keypoints);
		bowDE_sift.setVocabulary(dictionary);
    	bowDE_sift.compute(image, keypoints, image_histogram);
    }
    
    else
    {
		detector_surf->detect(image, keypoints);
		bowDE_surf.setVocabulary(dictionary);
    	bowDE_surf.compute(image, keypoints, image_histogram);
    }
    
    return image_histogram;
}

void ObjectDetector::FindCenter(Mat frame, Point2f* center_pos, int object_class)
{
	Mat descriptors_frame;
	FlannBasedMatcher matcher;
  	vector<DMatch> matches;
  	SiftDescriptorExtractor detector_sift;
  	SurfDescriptorExtractor detector_surf;
  	
  	if(descriptor_extractor == 0)
  		detector_sift.compute(frame, keypoints, descriptors_frame);
  	else
  		detector_surf.compute(frame, keypoints, descriptors_frame);
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
	//Passa filtro gaussiano
	if(blur_size > 0)
		GaussianBlur(frame, frame, Size(blur_size, blur_size), 0, 0);

	//Computa histograma do frame segundo BoF
	Mat frame_histogram = ComputeHistogram(frame);

	//Inpede crash de histogramas zerados
	if(frame_histogram.rows == 0 || frame_histogram.cols == 0)
	{
		Mat noncrash(1, dictionary_size, CV_32FC1, Scalar::all(0));
		frame_histogram = noncrash;
	}
	
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
