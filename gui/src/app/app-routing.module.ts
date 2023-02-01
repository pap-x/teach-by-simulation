import { NgModule } from '@angular/core';
import { Routes, RouterModule } from '@angular/router';
import { SetupComponent } from './setup/setup.component';
import { TeachComponent } from './teach/teach.component';

const routes: Routes = [
  { path: 'setup', component: SetupComponent },
  { path: '', component: SetupComponent },
  { path: 'teach', component: TeachComponent}
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }
