<!-- Slide number: 1 -->

![Generated image](Picture2.jpg)
Plant Disease Defect
2025.07.07

<!-- Slide number: 2 -->
AI Projects
🔬 Histopathology Image Analysis
Expected Goal: To design, develop, and clinically validate an AI-powered histopathology image analysis system for cancer diagnosis, utilizing whole slide images (WSIs).(목표: 전체 슬라이드 이미지(WSI)를 활용하여 암 진단을 위한 AI 기반 조직병리 이미지 분석 시스템을 설계, 개발 및 임상적으로 검증하는 것)
Stakeholder: KNU Hospital

🌿 Plant Disease Detection
Expected Goal: To build a scalable, vision-based AI system for automatic detection of diseases across various plant species, including but not limited to pear plants. The model will analyze visual symptoms on plant parts such as leaves, fruits, stems, and branches to detect early signs of biotic and abiotic stress.(목표: 배나무를 포함한 다양한 식물 종에 대해 질병을 자동으로 탐지할 수 있는 확장 가능한 비전 기반 AI 시스템을 구축하는 것. 이 모델은 잎, 열매, 줄기, 가지 등 식물의 다양한 부위에 나타나는 생물학적 및 비생물학적 스트레스의 초기 징후를 시각적으로 분석함)
Stakeholder: ?

🖼️ Intaglio Line Art Defect Detection
Expected Goal: To develop a deep learning-based system for automatic defect detection in engraved intaglio line art by learning the mapping between digital designs and their physical engravings.(목표: 디지털 디자인과 실제 인그레이빙 간의 매핑을 학습하여 인타글리오 선화(engraved intaglio line art)에서 결함을 자동으로 탐지하는 딥러닝 기반 시스템을 개발하는 것)
Stakeholder: KOMSCO

<!-- Slide number: 3 -->
🌿 Plant Disease Detection (Previous Goal)
Detect defects in pear plants (including leaves, fruits, stems, and branches) using a vision-based model.(비전 기반 모델을 활용하여 배나무의 잎, 열매, 줄기, 가지 등에 발생한 결함을 탐지합니다)
Public dataset from aihub.or.kr, which contains pear orchard images labeled as either normal or fireblight.(aihub.or.kr의 공개 데이터셋을 사용하였으며, 이 데이터셋은 배 과수원 이미지를 정상 또는 화상병으로 라벨링하고 있습니다)
Because the dataset does not label individual images as defective or not beyond the fireblight label, we explored using a vision-language model (VLM), such as Gemini, to help assess defect presence.(해당 데이터셋은 화상병 여부 외에 개별 이미지의 결함 유무를 세부적으로 라벨링하지 않기 때문에, 우리는 Gemini와 같은 비전-언어 모델(VLM)을 활용하여 결함 존재 여부를 평가하는 방법을 탐색했습니다)
To better leverage the VLM, we asked it to rate each image on a severity scale, where severity = 0 indicates a perfectly healthy plant (according to the VLM), and severity = 10 indicates a completely unhealthy plant.(VLM을 보다 효과적으로 활용하기 위해, 각 이미지에 대해 심각도(severity) 척도를 기준으로 평가하도록 요청했습니다. severity = 0: VLM 기준으로 완전히 건강한 식물; severity = 10: VLM 기준으로 완전히 병든 식물)

![A close up of a plant AI-generated content may be incorrect.](Picture17.jpg)

![A close up of a tree branch AI-generated content may be incorrect.](Picture20.jpg)

![A tree branch with green leaves AI-generated content may be incorrect.](Picture39.jpg)

![Cloud with solid fill](Graphic15.jpg)
aihub.or.kr
Fireblight
Normal

![A tree branch with green leaves AI-generated content may be incorrect.](Picture23.jpg)

![A tree branch with green leaves AI-generated content may be incorrect.](Picture26.jpg)

![A tree branch with green leaves AI-generated content may be incorrect.](Picture42.jpg)

![](Picture33.jpg)
Normal
Defect

<!-- Slide number: 4 -->
🌿 Plant Disease Detection
Prompt:
We used the following prompt:
Inspect this orchard image carefully. Evaluate the condition of the visible plant part.
If the plant part appears completely fresh, intact, and undamaged—no visible signs of drying, pest damage, holes, discoloration, rot, deformities, mold, or breakage—classify the image as **NORMAL**.
If any defects are present, classify the image as **DEFECT**. Then, assign a **severity score from 1 to 10**, where:
- **1** = Very minor, cosmetic, or negligible defect (e.g., a small dry spot)
- **5** = Moderate defect that affects part of the plant
- **10** = Severe, widespread, or critical damage
### Return the following format:
- Classification: `NORMAL` or `DEFECT`
- Severity: `0` for NORMAL, or `1–10` for DEFECT
- Brief explanation, including the defect type and the plant part
### Example Outputs:
- `NORMAL - severity 0 - fruit is healthy with no visible damage`
- `DEFECT - severity 2 - minor leaf tip browning`
- `DEFECT - severity 8 - fruit is partially rotten with pest holes`

<!-- Slide number: 5 -->
🌿 Plant Disease Detection (Previous Goal)
Model:
We train a Fast R-CNN, an object detection architecture, to detect and crop the part of the plant (e.g., stem or leaf). Then, we train a classification model (e.g., ResNet18) to classify as defect or not defect.

![A tree branch with green leaves AI-generated content may be incorrect.](Picture39.jpg)

![A tree branch with green leaves AI-generated content may be incorrect.](Picture6.jpg)
Fast R-CNN
ResNet18
Stem / Normal

Stem/ Normal

![A tree branch with green leaves AI-generated content may be incorrect.](Picture10.jpg)

![A tree branch with green leaves AI-generated content may be incorrect.](Picture42.jpg)

ResNet18
Fast R-CNN
Leaves / Defect

Leaves / Defect

![](Picture2.jpg)
Fast R-CNN Architecture

<!-- Slide number: 6 -->
🌿 Plant Disease Detection (Previous Goal)
Result:
The model was trained with 32461 training samples, 8119 samples for validation, and 5079 samples for testing; achieving an accuracy of around 93.7%.

![](Picture7.jpg)
High: Defect / Low: HealthyBlue words: Correct / Red words: Incorrect

<!-- Slide number: 7 -->
🌿 Plant Disease Detection - Fireblight Only
Classes: leaf w/ fireblight, leaf w/o fireblight, fruit w/ fireblight, fruit w/o fireblight
If trained to detect fireblight only, the model achieves an accuracy of around 99.1%, and F1-score of 99.47%.
Note: Only leaves and fruits contain the fireblight class (불마름병 등급은 잎과 열매에만 포함되어 있습니다).

![](Picture2.jpg)
Blue words: Correct / Red words: Incorrect

<!-- Slide number: 8 -->
🌿 Plant Disease Detection (Future Goal)
To build a vision-based AI system for automatic detection of diseases across various plant species:
Persimmon: Circular leaf spot, Leaf blotch, Anthracnose, Powdery mildew.
	감 : 둥근무늬 낙엽병, 모무늬병, 탄저병, 흰가루병
Citrus: Greasy spot, Canker, Green mold, Melanose, Resin disease, Gray mold, Anthracnose.
감귤 : 검은점무늬병, 궤양병, 녹색곰팡이병, 더뎅이병, 수지병, 잿빛곰팡이병, 탄저병
Pear: Black spot, Rust, Leaf spot, Fire blight, Powdery mildew, Sooty mold, Sclerotinia rot.
배 : 검은별무늬병, 붉은별무늬병, 잎검은점병, 화상병, 흰가루병, 그을음병, 균핵병
Peach: Bacterial shot hole, Phytophthora blight, Leaf curl, Gray mold, Gray leaf spot, Anthracnose, Powdery mildew, Leaf scorch.
	복숭아 : 세균 구멍병, 역병, 잎오갈병, 잿빛곰팡이병, 잿빛무늬병, 탄저병, 흰가루병, 잎마름병
Apple: Brown spot, Alternaria rot, Sooty blotch, White rot, Rust, Leaf spot, Anthracnose, Fire blight.
	사과 : 갈색무늬병, 겹무늬썩음병, 그을음점무늬병, 부란병, 붉은별무늬병, 점무늬낙엽병, 탄저병, 화상병

![](Picture6.jpg)

![](Picture38.jpg)
AI Model Training

![](Picture14.jpg)

![](Picture10.jpg)

![](Picture67.jpg)

![](Picture40.jpg)
Target: Over 90% accuracy

![](Picture61.jpg)

![](Picture15.jpg)

![](Picture30.jpg)
AI-based Image
Selection

![](Picture23.jpg)
Training Dataset
Web Dataset
Initial Dataset
Human Expert Rules & VLM-based Filtering
AI Development
Farmers Crowdsourcing

<!-- Slide number: 9 -->
🌿 Plant Disease Detection (Future Goal)

![](Picture6.jpg)

![](Picture38.jpg)
AI Model Training

![](Picture14.jpg)

![](Picture10.jpg)

![](Picture67.jpg)

![](Picture40.jpg)
Target: Over 90% accuracy

![](Picture61.jpg)

![](Picture15.jpg)

![](Picture30.jpg)
AI-based Image
Selection

![](Picture23.jpg)
Training Dataset
Web Dataset
Initial Dataset
(1) Human Expert Rules & VLM-based Filtering
(3) AI Development
(2) Farmers Crowdsourcing
(1) Human Expert Rules & VLM-based Filtering (전문가 규칙 및 VLM 기반 필터링)
Web Dataset Collection: Images are collected from the internet.(웹 데이터셋 수집: 인터넷에서 이미지를 수집합니다)
Filtering Process: The collected web data is filtered using:(필터링 과정: 수집된 웹 데이터를 다음 기준으로 필터링합니다)
Human Expert Rules: Domain-specific knowledge is used to eliminate irrelevant or low-quality data.(전문가 규칙: 분야별 전문 지식을 활용하여 관련 없는 데이터나 저품질 데이터를 제거합니다)
VLM-based Filtering: Vision-Language Models (VLMs), such as Gemini, are used to semantically filter and validate the relevance of the images.(VLM 기반 필터링: Gemini와 같은 비전-언어 모델(VLM)을 사용하여 이미지의 의미적 관련성을 필터링하고 검증합니다)
Output: A cleaned dataset is produced for the next phase. (결과물: 다음 단계에 사용할 정제된 데이터셋이 생성됩니다)
Duration: 2-4 weeks

<!-- Slide number: 10 -->
🌿 Plant Disease Detection (Future Goal)

![](Picture6.jpg)

![](Picture38.jpg)
AI Model Training

![](Picture14.jpg)

![](Picture10.jpg)

![](Picture67.jpg)

![](Picture40.jpg)
Target: Over 90% accuracy

![](Picture61.jpg)

![](Picture15.jpg)

![](Picture30.jpg)
AI-based Image
Selection

![](Picture23.jpg)
Training Dataset
Web Dataset
Initial Dataset
(1) Human Expert Rules & VLM-based Filtering
(3) AI Development
(2) Farmers Crowdsourcing
(2) Farmer Crowdsourcing (농민 크라우드소싱)
Initial Dataset Expansion: Farmers contribute by capturing and uploading images, typically from the field.(초기 데이터셋 확장: 농민들이 현장에서 직접 이미지를 촬영하고 업로드하여 데이터 수집에 기여합니다)
Result: These images are added to form the Initial Dataset, enhancing diversity and real-world representation.(결과: 이 이미지들은 초기 데이터셋에 추가되어 데이터의 다양성과 실제 현장 반영도를 향상시킵니다)
Duration: 4-8 weeks.

<!-- Slide number: 11 -->
🌿 Plant Disease Detection (Future Goal)

![](Picture6.jpg)

![](Picture38.jpg)
AI Model Training

![](Picture14.jpg)

![](Picture10.jpg)

![](Picture67.jpg)

![](Picture40.jpg)
Target: Over 90% accuracy

![](Picture61.jpg)

![](Picture15.jpg)

![](Picture30.jpg)
AI-based Image
Selection

![](Picture23.jpg)
Training Dataset
Web Dataset
Initial Dataset
(1) Human Expert Rules & VLM-based Filtering
(3) AI Development
(2) Farmers Crowdsourcing
(3) AI Development (AI 개발)
AI-based Image Selection: An AI algorithm (such as clustering) is used to automatically select the most relevant and high-quality images from the Initial Dataset.(AI 기반 이미지 선택: AI 알고리즘(예: 클러스터링)을 활용하여 초기 데이터셋에서 가장 관련성 높고 고품질의 이미지를 자동으로 선택합니다)
Training Dataset Creation: Selected images are compiled into the final Training Dataset.(학습용 데이터셋 생성: 선택된 이미지들을 최종 학습 데이터셋으로 구성합니다)
Model Training: This dataset is used to train an AI model.(모델 학습: 이 데이터셋을 사용하여 AI 모델을 학습시킵니다)
Target: The training aims to achieve over 90% accuracy in performance.(목표: 모델 성능이 90% 이상의 정확도를 달성하는 것을 목표로 합니다)
Duration: 3-6 weeks
