
#include <iostream>
#include <thread>
#include <unistd.h>
#include <list>
#include <string>
#include <vector>

using std::string;

class CKeyFrame;
std::list<CKeyFrame *> g_listKeyFrames;

// char szStr[2048000] = { 0 };

class CKeyFrame
{
public:    
    CKeyFrame() {}
    virtual ~CKeyFrame() {}

    std::list<string> list_keys;
    std::list<char> list_keys2;
    int a[10240];
};

class CLoopClosure
{
public:
    CLoopClosure() {}
    virtual ~CLoopClosure() {}
    void command();
    std::list<CKeyFrame *> m_listKeyFrames;
    std::list<CKeyFrame> m_listKeyFrames2;
};

void CLoopClosure::command()
{
  std::cout << "loop closure" << std::endl;
  std::cout << "1 command()" << std::endl;
  while (true)
  {
  #if 1//def _KEY_PRESS_
    // char c = getchar();
    // std::string strInput;
    // std::cin >> strInput;
    char c;
    std::cin >> c;
    
    if(c == 'a')
    {
        std::cout << "press 'a' to add keyframe" << std::endl;
        CKeyFrame *frame = new CKeyFrame();

        /*FILE *fp = fopen("../stereo3_20240920.log", "r");
        if(fp)
        {
            while(!feof(fp))
            {
                char buff[1024] = "";
                fread(buff, 1, 1023, fp);
                frame->list_keys.emplace_back(buff);
                // std::cout << buff << std::endl;
            }
            fclose(fp);
        }*/
        
        // string str;
        // str.resize(1024);
        for(int i = 0; i < 1024 * 4; i++)
        // frame->list_keys.emplace_back("aaa");
        frame->list_keys2.emplace_back('a');
        // std::cout << "1=" << sizeof(frame) << std::endl;
        // std::cout << "2=" << sizeof(CKeyFrame) << std::endl;
        m_listKeyFrames.push_back(frame);
        std::cout << "key frame list size :" << m_listKeyFrames.size() << std::endl;
    }

    else if(c == 'r')
    {
        std::cout << "press 'r' to remove a keyframe" << std::endl;
        auto iter = m_listKeyFrames.begin();
        
        if(iter != m_listKeyFrames.end())
        {
            // std::cout << (*iter)->list_keys.front().size() << std::endl;
            std::cout << (*iter)->list_keys.front() << std::endl;
            delete *iter;
            m_listKeyFrames.erase(iter);
            std::cout << "key frame list size :" << m_listKeyFrames.size() << std::endl;
        }
    }

    else if(c == 'c')
    {
        std::cout << "press 'c' to clear all" << std::endl;

        auto iter = m_listKeyFrames.begin();
        for( ; iter != m_listKeyFrames.end(); iter++)
        {
            (*iter)->list_keys.clear();
            delete *iter;
        }
        m_listKeyFrames.clear();
        std::cout << "key frame list size :" << m_listKeyFrames.size() << std::endl;
    }
    else if(c == 'q')
    {
        std::cout << "press 'q' to quit" << std::endl;
        // Stop();
        // shutdown = true;
        break ;

    }

    else if(c == 'b')
    {
        std::cout << "press 'b' to add keyframe" << std::endl;
        CKeyFrame frame;

        /*FILE *fp = fopen("../stereo3_20240920.log", "r");
        if(fp)
        {
            while(!feof(fp))
            {
                char buff[1024] = "";
                fread(buff, 1, 1023, fp);
                frame->list_keys.emplace_back(buff);
                // std::cout << buff << std::endl;
            }
            fclose(fp);
        }*/
        
        // string str;
        // str.resize(1024);
        for(int i = 0; i < 1024 * 4; i++)
        // frame->list_keys.emplace_back("bbbbbbbbb");
        frame.list_keys2.emplace_back('b');
        // std::cout << "1=" << sizeof(frame) << std::endl;
        // std::cout << "2=" << sizeof(CKeyFrame) << std::endl;
        m_listKeyFrames2.push_back(frame);
        std::cout << "key frame list size :" << m_listKeyFrames2.size() << std::endl;
    }

    else if(c == 'e')
    {
        std::cout << "press 'r' to remove a keyframe" << std::endl;
        auto iter = m_listKeyFrames2.begin();
        
        if(iter != m_listKeyFrames2.end())
        {
            // std::cout << (*iter)->list_keys.front().size() << std::endl;
            std::cout << (*iter).list_keys2.front() << std::endl;
            (*iter).list_keys2.clear();
            m_listKeyFrames2.erase(iter);
            std::cout << "key frame list size :" << m_listKeyFrames2.size() << std::endl;
        }
    }

    else if(c == 'f')
    {
        std::cout << "press 'c' to clear all" << std::endl;

        auto iter = m_listKeyFrames2.begin();
        for( ; iter != m_listKeyFrames2.end(); iter++)
        {
            (*iter).list_keys2.clear();
            // delete *iter;
        }
        m_listKeyFrames2.clear();
        // m_listKeyFrames2.shrink_to_fit();
        std::cout << "key frame list size :" << m_listKeyFrames2.size() << std::endl;
    }

    

  #endif

    std::chrono::milliseconds dura(500);
    std::this_thread::sleep_for(dura);
  }

  std::cout << "2 command()" << std::endl;
}

void command(void *pParam) //()
{
  //
  std::cout << "1 command()" << std::endl;
  while (true)
  {
  #if 1//def _KEY_PRESS_
    // char c = getchar();
    // std::string strInput;
    // std::cin >> strInput;
    char c;
    std::cin >> c;
    
    if(c == 'a')
    {
        std::cout << "press 'a' to add keyframe" << std::endl;
        CKeyFrame *frame = new CKeyFrame();

        FILE *fp = fopen("../stereo3_20240920.log", "r");
        if(fp)
        {
            while(!feof(fp))
            {
                char buff[1024] = "";
                fread(buff, 1, 1023, fp);
                frame->list_keys.emplace_back(buff);
                // std::cout << buff << std::endl;
            }
            fclose(fp);
        }
        
        // string str;
        // str.resize(1024);
        // for(int i = 0; i < 1024; i++)
        // frame->list_keys.emplace_back("aaa");
        // std::cout << "1=" << sizeof(frame) << std::endl;
        // std::cout << "2=" << sizeof(CKeyFrame) << std::endl;
        g_listKeyFrames.push_back(frame);
        std::cout << "key frame list size :" << g_listKeyFrames.size() << std::endl;
    }

    else if(c == 'r')
    {
        std::cout << "press 'r' to remove a keyframe" << std::endl;
        auto iter = g_listKeyFrames.begin();
        
        if(iter != g_listKeyFrames.end())
        {
            // std::cout << (*iter)->list_keys.front().size() << std::endl;
            std::cout << (*iter)->list_keys.front() << std::endl;
            delete *iter;
            g_listKeyFrames.erase(iter);
            std::cout << "key frame list size :" << g_listKeyFrames.size() << std::endl;
        }
    }

    else if(c == 'c')
    {
        std::cout << "press 'c' to clear all" << std::endl;

        auto iter = g_listKeyFrames.begin();
        for( ; iter != g_listKeyFrames.end(); iter++)
        {
            (*iter)->list_keys.clear();
            delete *iter;
        }
        g_listKeyFrames.clear();
    }
    else if(c == 'q')
    {
        std::cout << "press 'q' to quit" << std::endl;
        // Stop();
        // shutdown = true;
        break ;

    }

    

  #endif

    std::chrono::milliseconds dura(500);
    std::this_thread::sleep_for(dura);
  }

  std::cout << "2 command()" << std::endl;
}

int main(int argc, char *argv[])
{
    std::list<char> listTmp;
    getchar();
    getchar();
    for(int i = 0; i < 1024 * 4; i++)
        listTmp.emplace_back('b');
    getchar();
    getchar();
    for(int i = 0; i < 1024 * 4; i++)
        listTmp.emplace_back('b');
    getchar();
    getchar();
    for(int i = 0; i < 1024 * 4; i++)
        listTmp.emplace_back('b');
    getchar();
    getchar();
    // std::list<char>(listTmp).swap(listTmp);
    // std::list<char>().swap(listTmp);
    listTmp.clear();
    std::cout << "clear" << std::endl;
    getchar();
    getchar();
    return 0;

    std::vector<int> vecTmp;
    vecTmp.clear();
    vecTmp.shrink_to_fit();
    

    // std::thread keyboard_command_process;
    // keyboard_command_process = std::thread(command, nullptr);
    // keyboard_command_process.join(); // terminate called without an active exception

    {
        std::thread keyboard_command_process;
        CLoopClosure *loop = new CLoopClosure();
        // keyboard_command_process = std::thread(&CLoopClosure::command, &loop);
        keyboard_command_process = std::thread(&CLoopClosure::command, loop);
        keyboard_command_process.join();
        delete loop;
    }
    

    
    while(1)
    {
        char ch=getchar();
        std::cout << "ch=" << (int)ch << std::endl;
        if(ch == 'q') break ;
        std::chrono::milliseconds dura(500);
        std::this_thread::sleep_for(dura);
    }

    // std::cout << "111" << std::endl;
    // system("pause");
    // std::cout << "222" << std::endl;

    return 0;
}