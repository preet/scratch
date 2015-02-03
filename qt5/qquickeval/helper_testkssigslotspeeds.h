#ifndef HELPER_TEST_KS_SIG_SLOT_SPEED_H
#define HELPER_TEST_KS_SIG_SLOT_SPEED_H

#include "helper.h"

#include <ks/KsObject.h>
#include <ks/KsSignal.h>
#include <ks/KsThread.h>

namespace ks
{
    namespace test
    {
        class Emitter : public Object
        {
        public:
            Emitter(shared_ptr<Thread> const &thread) :
                Object(thread)
            {

            }

            ~Emitter()
            {

            }

            Signal<Thing> SignalThing;
            Signal<Thread*> SignalStopThread;
        };


        class Receiver : public Object
        {
        public:
            Receiver(shared_ptr<Thread> const &thread) :
                Object(thread),
                invoke_count(0)
            {

            }

            ~Receiver()
            {

            }

            void SlotThing(Thing const &)
            {
                invoke_count++;
            }

            void SlotStopThread(Thread * thread)
            {
                thread->Stop();
            }

            uint invoke_count;
        };

    } // test

} // ks


class HelperTestKsSigSlotSpeed : public Helper
{
    Q_OBJECT

public:
    explicit HelperTestKsSigSlotSpeed(QQuickView * qqview,QObject *parent=0);
    void runTest();
};


#endif // HELPER_TEST_KS_SIG_SLOT_SPEED_H
