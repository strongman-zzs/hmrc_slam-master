
#include <hmrc_slam/client/ClientSystem.h>

namespace hmrc_slam{

ClientSystem::ClientSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile, const string &strCamFile)
    : mNh(Nh), mNhPrivate(NhPrivate), mstrCamFile(strCamFile), mpUID(new estd::UniqueIdDispenser())
{    
    params::ShowParams();

    //+++++ load params +++++

//    std::string TopicNameCamSub;
    int ClientId;

    mNhPrivate.param("ClientId",ClientId,-1);
    mClientId = static_cast<size_t>(ClientId);

    //+++++ Check settings files +++++

    cv::FileStorage fsSettingsCam(strCamFile.c_str(), cv::FileStorage::READ);
    if(!fsSettingsCam.isOpened())
    {
       cerr << "Failed to open cam file at: " << strCamFile << endl;
       exit(-1);
    }

    //+++++ load vocabulary +++++

    this->LoadVocabulary(strVocFile);

    //+++++ Create KeyFrame Database +++++
    mpKFDB.reset(new KeyFrameDatabase(mpVoc));

    //+++++ Create the Map +++++
    mpMap.reset(new Map(mNh,mNhPrivate,mClientId,eSystemState::CLIENT));
    usleep(10000); //wait to avoid race conditions
    //+++++ Initialize Agent +++++
    mpAgent.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpKFDB,mpMap,mClientId,mpUID,eSystemState::CLIENT,strCamFile,nullptr));
    usleep(10000); //wait to avoid race conditions
    mpAgent->InitializeThreads();
    usleep(10000); //wait to avoid race conditions

    //++++++++++
    cout << endl << "Clientsystem initialized (Client ID: " << mClientId << ")" << endl;
}

void ClientSystem::LoadVocabulary(const string &strVocFile)
{
    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVoc.reset(new ORBVocabulary());
    bool bVocLoad = mpVoc->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;
}

} //end namespace
