import { Component, OnInit, AfterViewInit } from '@angular/core';
import { AssemblyService } from '../assembly.service';
import { Assembly } from '../assembly.model';
import { RosService } from '../ros.service';
import { MatSnackBar } from '@angular/material/snack-bar';

@Component({
  selector: 'app-teach',
  templateUrl: './teach.component.html',
  styleUrls: ['./teach.component.css']
})
export class TeachComponent implements OnInit {

  assembly: Assembly;
  selectedKf = "0";
  saveDisabled = false;
  simStatus = 'home';
  movingParts = [];
  selectedMoving = "0";
  iframe: Window;

  constructor(private assService: AssemblyService,
              private ros: RosService,
              private snackBar: MatSnackBar) {
    this.assembly = assService.assembly;
  }

  ngOnInit(): void {
    // Set moving parts array to display the select moving parts module
    for (const [index, object] of this.assembly.objects.entries()) {
      if (!object.static) {
        this.movingParts.push({name: object.name, index: index+1})
      }
    }

    // Get part positions and set to initial keyframe
    console.log(this.assembly);
    this.ros.getPart("Green PCB new", (data) => {
      this.assembly.keyframes[0].object_poses[this.assembly.objects[0].name] = data.pose;
      this.ros.getPart("Yellow PCB new", (data) => {
        this.assembly.keyframes[0].object_poses[this.assembly.objects[1].name] = data.pose;
        if (this.assembly.object_number===3) {
          this.ros.getPart("TV new", (data) => {
            this.assembly.keyframes[0].object_poses[this.assembly.objects[2].name] = data.pose;
            console.log(this.assembly);
            // Save to initial keyframe
            //this.assembly.keyframes[0].object_poses = {'green_pcb': object_poses["green_pcb"].pose, 'yellow_pcb': object_poses["yellow_pcb"].pose, 'tv': object_poses["tv"].pose};
          })
        }
      })
    });

    this.ros.simStatus().subscribe((result) => {
      if (result.data=='finish') {
        this.simStatus = 'finish';
      }
      else if (result.data=='home') {
        this.simStatus = 'home';
        // return objects to selected kf positions
        this.returnObjects();
      }
    })
  }

  ngAfterViewInit(): void{
    this.iframe = (document.getElementById('gzweb-frame') as HTMLFrameElement).contentWindow;
  }

  onSelectKf(): void {

    // show moving part
    this.selectedMoving = this.assembly.keyframes[+this.selectedKf].moving_part.toString();
    // If the object poses obj is not empty
    if (Object.keys(this.assembly.keyframes[+this.selectedKf].object_poses).length != 0) {
      // let outer_this = this;
      // Maybe check if the parts are not static first
      this.ros.movePart('Green PCB new', this.assembly.keyframes[+this.selectedKf].object_poses[this.assembly.objects[0].name], ()=>{
        this.ros.movePart('Yellow PCB new', this.assembly.keyframes[+this.selectedKf].object_poses[this.assembly.objects[1].name]);
        // this.save_disabled = false;
      });
    }
  }

  isEmptyObject(obj) {
    return (Object.keys(obj).length === 0);
  }

  onAdd(): void {

  }

  onRefresh(): void {
    this.ros.goHome();

  }

  returnObjects(): void {
    // Move objects to selected Kf position
    this.ros.movePart('Green PCB new', this.assembly.keyframes[+this.selectedKf].object_poses[this.assembly.objects[0].name], ()=>{
      this.ros.movePart('Yellow PCB new', this.assembly.keyframes[+this.selectedKf].object_poses[this.assembly.objects[1].name]);
      // this.save_disabled = false;
    });
  }

  onSave(): void {
    // save moving part
    this.assembly.keyframes[+this.selectedKf].moving_part = +this.selectedMoving;
    // save positions on local array and on remote
    let object_poses = {};
    this.iframe.postMessage('cursor', '*');
    setTimeout(()=> {
      this.ros.getPart("Green PCB new", (data) => {
        this.assembly.keyframes[+this.selectedKf].object_poses[this.assembly.objects[0].name] = data.pose;
        this.ros.getPart("Yellow PCB new", (data) => {
          this.assembly.keyframes[+this.selectedKf].object_poses[this.assembly.objects[1].name] = data.pose;
          if (this.assembly.object_number===2) {
            const positions_saved = this.snackBar.open('Positions saved!', 'OK', {
              duration: 3000
            });
          }
          else if (this.assembly.object_number===3) {
            this.ros.getPart("TV new", (data) => {
              this.assembly.keyframes[+this.selectedKf].object_poses[this.assembly.objects[2].name] = data.pose;

              // Save to selected keyframe
              //this.assembly.keyframes[+this.selectedKf].object_poses = {green_pcb: object_poses["green_pcb"].pose, yellow_pcb: object_poses["yellow_pcb"].pose, tv: object_poses["tv"].pose};
              const positions_saved = this.snackBar.open('Positions saved!', 'OK', {
                duration: 3000
              });
            });
          }
        });
      });
    }, 500);
  }

  onPlayAnim(): void {
    // Save assembly json file
    this.assService.uploadJson(this.assembly).subscribe((result) => {
      console.log(result.message);
      if (result.message=='success') {
        this.simStatus = 'playing';
        const json_saved = this.snackBar.open('Keyframes saved, animation starting now!', 'OK', {
          duration: 3000
        });

        // Move parts to initial position and begin the simulation

        this.ros.movePart('Green PCB new', this.assembly.keyframes[0].object_poses[this.assembly.objects[0].name], ()=>{
          this.ros.movePart('Yellow PCB new', this.assembly.keyframes[0].object_poses[this.assembly.objects[1].name], () => {
            this.ros.playSimulation();
          });
        });
      }
      else {
        const json_error = this.snackBar.open('There was an error while saving the keyframes', 'OK', {
          duration: 3000
        });
      }

    })

  }

  edit(mode: string) {

    // Use the appropriate editing mode in gzweb
    if (mode==='pose') {
      this.iframe.postMessage('pose', '*');
    }
    else if (mode==='orient') {
      this.iframe.postMessage('orient', '*');
    }
  }

  onPauseAnim(): void {

  }

  onReloadAnim(): void {

  }

}
