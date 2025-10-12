#include "buzzer_command.hpp"

namespace control
{
namespace buzzer
{
    BuzzerCommand::BuzzerCommand(
        BuzzerSubsystem *const buzzer, 
        src::Drivers *drivers,
        tap::arch::MilliTimeout delayTimer, 
        uint16_t** notes,
        uint8_t songLength,
        uint8_t readIndex)
        : buzzer(buzzer), 
          drivers(drivers),
          delayTimer(delayTimer),
          notes(notes),
          songLength(songLength),
          readIndex(0),
          songFinished(false)
    {
        if(buzzer == nullptr){
            return;
        }
        this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(buzzer));
    }

    void BuzzerCommand::initialize(){
        delayTimer.restart(0);
    }

    void BuzzerCommand::execute(){
        if(readIndex>= songLength){
            songFinished = true;
        }
        if(delayTimer.execute()){
            delayTimer.restart(notes[readIndex][NOTE_DURATION_INDEX]);
            buzzer->playNote(notes[readIndex][MIDI_NOTE_INDEX]);
            readIndex++;
        }
    }

    bool BuzzerCommand::isFinished() const{
        return songFinished;
    }

    void BuzzerCommand::end(bool){
        buzzer->stopSound();
    }

    const char*  BuzzerCommand::getName() const{
        return "buzzer command";
    }

}
}
