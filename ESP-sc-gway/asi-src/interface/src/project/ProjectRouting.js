import React, { Component } from 'react';
import { Redirect, Switch } from 'react-router';

import { PROJECT_PATH } from '../constants/Env';
import AuthenticatedRoute from '../authentication/AuthenticatedRoute';
import DemoProject from './DemoProject';
import LoraSection from './LoraSection';
import GPRSSection from './gprs/GPRSSection';
import PFSection from './packet_forwarder/PFSection';

class ProjectRouting extends Component {

  render() {
    return (
      <Switch>
        {
          /*
          * Add your project page routing below.
            <AuthenticatedRoute exact path={`/${PROJECT_PATH}/demo/*`} component={DemoProject} />
          */
        }
        <AuthenticatedRoute exact path={`/${PROJECT_PATH}/lora/*`} component={LoraSection} />
        <AuthenticatedRoute exact path={`/${PROJECT_PATH}/pf/*`} component={PFSection} />
        <AuthenticatedRoute exact path={`/${PROJECT_PATH}/gprs/*`} component={GPRSSection} />
        {
          /*
          * The redirect below caters for the default project route and redirecting invalid paths.
          * The "to" property must match one of the routes above for this to work correctly.
          */
        }

        <Redirect to={`/${PROJECT_PATH}/lora/`} />
      </Switch>
    )
  }

}

export default ProjectRouting;
